#!/usr/bin/env python3
"""Terminal UI for sending drawing commands to the delta robot."""

import math
import sys
import time

from ros_bootstrap import prepare_ros_python_environment
from ui_config import (
    DEFAULT_AMAX,
    DEFAULT_VMAX,
    DRAWING_COMPLETION_TIMEOUT_SEC,
    DRAWING_START_TIMEOUT_SEC,
    MIN_AMAX,
    MIN_VMAX,
    ShapeDefinition,
    TERMINAL_SHAPE_OPTIONS,
    shape_message_type,
)

prepare_ros_python_environment(__file__)

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from my_delta_robot.msg import Posicionxyz, VmaxAmax  # noqa: E402


class UserInterfaceNode(Node):
    """Simple terminal menu that publishes existing command topics."""

    def __init__(self) -> None:
        super().__init__("user_interface_node")
        self.motion_limits_pub = self.create_publisher(VmaxAmax, "set_vmax_amax", 10)
        self.target_action_pub = self.create_publisher(Posicionxyz, "set_current_point", 10)
        self.create_subscription(String, "drawing_status", self._on_drawing_status, 10)
        self.drawing_busy = False
        self.drawing_started = False
        self.drawing_result: tuple[bool, str] | None = None
        self.expected_shape: str | None = None

    @staticmethod
    def _status_shape_name(status: str) -> str:
        detail = status.partition(":")[2].strip()
        return detail.split(" - ", maxsplit=1)[0]

    def _on_drawing_status(self, msg: String) -> None:
        shape_name = self._status_shape_name(msg.data)
        if msg.data.startswith("STARTED:"):
            if self.expected_shape is not None and shape_name != self.expected_shape:
                self.get_logger().warn(
                    f"Expected {self.expected_shape}, but an external "
                    f"{shape_name} drawing started."
                )
            self.expected_shape = shape_name
            self.drawing_busy = True
            self.drawing_started = True
            self.drawing_result = None
            self.get_logger().info(msg.data)
        elif msg.data.startswith(("DONE:", "FAILED:")):
            if self.expected_shape is None:
                self.get_logger().warn(f"Untracked drawing status: {msg.data}")
                return
            if shape_name != self.expected_shape:
                self.get_logger().warn(
                    f"Ignored status for {shape_name}; waiting for "
                    f"{self.expected_shape}."
                )
                return
            succeeded = msg.data.startswith("DONE:")
            self.drawing_busy = False
            self.drawing_started = False
            self.drawing_result = (succeeded, msg.data)
            if succeeded:
                self.get_logger().info(msg.data)
            else:
                self.get_logger().error(msg.data)
        else:
            self.get_logger().warn(f"Ignored drawing status: {msg.data}")

    def poll_status(self, max_callbacks: int = 3) -> None:
        for _ in range(max_callbacks):
            rclpy.spin_once(self, timeout_sec=0.0)

    def wait_for_subscribers(self, timeout_sec: float = 2.0) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if (
                self.motion_limits_pub.get_subscription_count() > 0
                and self.target_action_pub.get_subscription_count() > 0
                and self.count_publishers("drawing_status") > 0
            ):
                return True

        if self.motion_limits_pub.get_subscription_count() == 0:
            self.get_logger().warn(
                "No subscriber on /set_vmax_amax yet. Start main_node to apply motion limits."
            )
        if self.target_action_pub.get_subscription_count() == 0:
            self.get_logger().warn(
                "No subscriber on /set_current_point yet. Start draw_node to execute drawings."
            )
        if self.count_publishers("drawing_status") == 0:
            self.get_logger().warn(
                "No publisher on /drawing_status yet. Completion cannot be verified."
            )
        return False

    def send_command(
        self, vmax: float, amax: float, shape_type: int, shape_name: str
    ) -> bool:
        self.poll_status()
        if self.drawing_busy:
            self.get_logger().warn(
                "Drawing command not sent because another drawing is active."
            )
            return False

        limits_msg = VmaxAmax()
        limits_msg.vmax = vmax
        limits_msg.amax = amax

        action_msg = Posicionxyz()
        action_msg.x0 = 0.0
        action_msg.y0 = 0.0
        action_msg.z0 = 0.0
        action_msg.type = shape_type

        self.drawing_busy = True
        self.drawing_started = False
        self.drawing_result = None
        self.expected_shape = shape_name
        self.motion_limits_pub.publish(limits_msg)
        rclpy.spin_once(self, timeout_sec=0.1)
        if self.drawing_started:
            self.get_logger().warn(
                "Drawing command not sent because an external drawing started."
            )
            return False
        self.target_action_pub.publish(action_msg)
        return True

    def wait_for_drawing_result(
        self,
        start_timeout_sec: float = DRAWING_START_TIMEOUT_SEC,
        completion_timeout_sec: float = DRAWING_COMPLETION_TIMEOUT_SEC,
    ) -> tuple[bool, str] | None:
        start_deadline = time.monotonic() + start_timeout_sec
        completion_deadline: float | None = None

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.drawing_result is not None:
                result, self.drawing_result = self.drawing_result, None
                self.drawing_started = False
                self.expected_shape = None
                return result

            now = time.monotonic()
            if self.drawing_started:
                if completion_deadline is None:
                    completion_deadline = now + completion_timeout_sec
                if now >= completion_deadline:
                    self.get_logger().error(
                        "Timed out waiting for /drawing_status completion."
                    )
                    return None
            elif now >= start_deadline:
                self.get_logger().error(
                    "No STARTED response arrived on /drawing_status."
                )
                return None

        return None


def read_float(prompt: str, default: float, minimum: float) -> float:
    while True:
        raw_value = input(f"{prompt} [{default:g}]: ").strip()
        if raw_value == "":
            return default

        try:
            value = float(raw_value)
        except ValueError:
            print("Please enter a numeric value.")
            continue

        if not math.isfinite(value) or value < minimum:
            print(f"Value must be finite and at least {minimum:g}.")
            continue

        return value


def read_shape() -> ShapeDefinition:
    while True:
        print("\nTarget drawing action:")
        for option, shape in TERMINAL_SHAPE_OPTIONS.items():
            print(f"  {option}. {shape.label}")

        raw_value = input("Select shape [1-3]: ").strip()
        if raw_value in TERMINAL_SHAPE_OPTIONS:
            return TERMINAL_SHAPE_OPTIONS[raw_value]

        print("Please choose 1, 2, or 3.")


def read_confirmation(vmax: float, amax: float, shape_name: str) -> bool:
    print("\nCommand summary:")
    print(f"  vmax:  {vmax:g} mm/s")
    print(f"  amax:  {amax:g} mm/s^2")
    print(f"  shape: {shape_name}")

    while True:
        raw_value = input("Send this command? [y/N]: ").strip().lower()
        if raw_value in ("y", "yes"):
            return True
        if raw_value in ("", "n", "no"):
            return False
        print("Please enter y or n.")


def report_drawing_result(result: tuple[bool, str]) -> None:
    success, message = result
    outcome = "completed" if success else "failed"
    print(f"Drawing {outcome}: {message}")


def run_menu(node: UserInterfaceNode) -> None:
    vmax = DEFAULT_VMAX
    amax = DEFAULT_AMAX

    print("\nDelta Robot User Interface")
    print("Publishes /set_vmax_amax and /set_current_point.")

    while rclpy.ok():
        node.poll_status()
        if node.drawing_busy:
            print("\nA drawing is active; waiting for its completion before continuing.")
            result = node.wait_for_drawing_result()
            if result is None:
                print(
                    "Drawing state is uncertain. Exiting without sending another command."
                )
                return
            report_drawing_result(result)

        print("\nMenu:")
        print("  1. Create drawing command")
        print("  2. Exit")
        choice = input("Select option [1-2]: ").strip()

        if choice == "2":
            print("Exiting user interface.")
            return
        if choice != "1":
            print("Please choose 1 or 2.")
            continue

        vmax = read_float("Input vmax in mm/s", vmax, MIN_VMAX)
        amax = read_float("Input amax in mm/s^2", amax, MIN_AMAX)
        shape = read_shape()

        if not read_confirmation(vmax, amax, shape.label):
            print("Command cancelled.")
            continue

        if not node.wait_for_subscribers():
            print("Command not sent because the required ROS endpoints are unavailable.")
            continue
        node.poll_status()
        if node.drawing_busy:
            print("Command not sent because another drawing is active.")
            continue

        shape_type = shape_message_type(shape, Posicionxyz)
        if not node.send_command(vmax, amax, shape_type, shape.key):
            print("Command not sent.")
            continue
        node.get_logger().info(
            f"Sent {shape.label.lower()} command with vmax={vmax:.1f} mm/s "
            f"and amax={amax:.1f} mm/s^2"
        )
        result = node.wait_for_drawing_result()
        if result is None:
            print("Drawing state is uncertain. Exiting without sending another command.")
            return
        report_drawing_result(result)


def main() -> int:
    rclpy.init()
    node = UserInterfaceNode()

    try:
        run_menu(node)
    except (KeyboardInterrupt, EOFError):
        print("\nExiting user interface.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(main())
