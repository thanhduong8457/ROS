#!/usr/bin/env python3
"""Terminal UI for sending drawing commands to the delta robot."""

import os
import shutil
import sys
import time
import ctypes


def relaunch_with_ros_python() -> None:
    """Use the Python interpreter that launched ros2 when the shell uses another python3."""
    ensure_ros_library_paths()

    if os.environ.get("MY_DELTA_UI_REEXEC") == "1":
        return

    ros2_path = shutil.which("ros2")
    if not ros2_path:
        return

    try:
        with open(ros2_path, "r", encoding="utf-8") as ros2_file:
            shebang = ros2_file.readline().strip()
    except OSError:
        return

    if not shebang.startswith("#!"):
        return

    ros_python = shebang[2:].split()[0]
    if not os.path.exists(ros_python) or os.path.realpath(ros_python) == os.path.realpath(sys.executable):
        return

    os.environ["MY_DELTA_UI_REEXEC"] = "1"
    os.execv(ros_python, [ros_python, __file__, *sys.argv[1:]])


def ensure_ros_library_paths() -> None:
    """Restore workspace library paths that macOS may strip from script shebang launches."""
    prefixes = get_ament_prefixes()
    lib_dirs = [
        os.path.join(prefix, "lib")
        for prefix in prefixes
        if os.path.isdir(os.path.join(prefix, "lib"))
    ]

    for env_name in ("DYLD_LIBRARY_PATH", "LD_LIBRARY_PATH"):
        current_paths = [
            path
            for path in os.environ.get(env_name, "").split(os.pathsep)
            if path
        ]
        merged_paths = []
        for path in [*lib_dirs, *current_paths]:
            if path not in merged_paths:
                merged_paths.append(path)
        if merged_paths:
            os.environ[env_name] = os.pathsep.join(merged_paths)


def get_ament_prefixes() -> list[str]:
    return [
        prefix
        for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep)
        if prefix
    ]


def ensure_workspace_log_dir() -> None:
    """Use the workspace log directory when ROS_LOG_DIR is not explicitly set."""
    if os.environ.get("ROS_LOG_DIR"):
        return

    for prefix in get_ament_prefixes():
        if os.path.basename(prefix) != "my_delta_robot":
            continue

        workspace_dir = os.path.dirname(os.path.dirname(prefix))
        log_dir = os.path.join(workspace_dir, "log")
        try:
            os.makedirs(log_dir, exist_ok=True)
        except OSError:
            return
        os.environ["ROS_LOG_DIR"] = log_dir
        return


def preload_ros_type_support_libraries() -> None:
    """Load package dylibs by absolute path before ROS requests them by name."""
    if sys.platform != "darwin":
        return

    library_names = [
        "libmy_delta_robot__rosidl_generator_c.dylib",
        "libmy_delta_robot__rosidl_typesupport_c.dylib",
        "libmy_delta_robot__rosidl_typesupport_fastrtps_c.dylib",
        "libmy_delta_robot__rosidl_typesupport_introspection_c.dylib",
        "libmy_delta_robot__rosidl_generator_py.dylib",
    ]
    load_mode = getattr(ctypes, "RTLD_GLOBAL", 0)

    for prefix in get_ament_prefixes():
        lib_dir = os.path.join(prefix, "lib")
        for library_name in library_names:
            library_path = os.path.join(lib_dir, library_name)
            if not os.path.exists(library_path):
                continue
            try:
                ctypes.CDLL(library_path, mode=load_mode)
            except OSError:
                pass


relaunch_with_ros_python()
ensure_workspace_log_dir()
preload_ros_type_support_libraries()

import rclpy
from rclpy.node import Node

from my_delta_robot.msg import Posicionxyz, VmaxAmax


DEFAULT_VMAX = 5000.0
DEFAULT_AMAX = 100.0
MIN_VMAX = 1.0
MIN_AMAX = 1.0

SHAPES = {
    "1": ("triangle", 7),
    "2": ("rectangle", 6),
    "3": ("circle", 8),
}


class UserInterfaceNode(Node):
    """Simple terminal menu that publishes existing command topics."""

    def __init__(self) -> None:
        super().__init__("user_interface_node")
        self.motion_limits_pub = self.create_publisher(VmaxAmax, "set_vmax_amax", 10)
        self.target_action_pub = self.create_publisher(Posicionxyz, "set_current_point", 10)

    def wait_for_subscribers(self, timeout_sec: float = 2.0) -> None:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if (
                self.motion_limits_pub.get_subscription_count() > 0
                and self.target_action_pub.get_subscription_count() > 0
            ):
                return
            rclpy.spin_once(self, timeout_sec=0.05)

        if self.motion_limits_pub.get_subscription_count() == 0:
            self.get_logger().warn(
                "No subscriber on /set_vmax_amax yet. Start main_node to apply motion limits."
            )
        if self.target_action_pub.get_subscription_count() == 0:
            self.get_logger().warn(
                "No subscriber on /set_current_point yet. Start draw_node to execute drawings."
            )

    def send_command(self, vmax: float, amax: float, shape_type: int) -> None:
        limits_msg = VmaxAmax()
        limits_msg.vmax = vmax
        limits_msg.amax = amax

        action_msg = Posicionxyz()
        action_msg.x0 = 0.0
        action_msg.y0 = 0.0
        action_msg.z0 = 0.0
        action_msg.type = shape_type

        for _ in range(3):
            self.motion_limits_pub.publish(limits_msg)
            rclpy.spin_once(self, timeout_sec=0.05)

        time.sleep(0.15)
        self.target_action_pub.publish(action_msg)
        rclpy.spin_once(self, timeout_sec=0.05)


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

        if value < minimum:
            print(f"Value must be at least {minimum:g}.")
            continue

        return value


def read_shape() -> tuple[str, int]:
    while True:
        print("\nTarget drawing action:")
        for option, (name, _) in SHAPES.items():
            print(f"  {option}. {name}")

        raw_value = input("Select shape [1-3]: ").strip()
        if raw_value in SHAPES:
            return SHAPES[raw_value]

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


def run_menu(node: UserInterfaceNode) -> None:
    vmax = DEFAULT_VMAX
    amax = DEFAULT_AMAX

    print("\nDelta Robot User Interface")
    print("Publishes /set_vmax_amax and /set_current_point.")

    while rclpy.ok():
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
        shape_name, shape_type = read_shape()

        if not read_confirmation(vmax, amax, shape_name):
            print("Command cancelled.")
            continue

        node.wait_for_subscribers()
        node.send_command(vmax, amax, shape_type)
        node.get_logger().info(
            f"Sent {shape_name} command with vmax={vmax:.1f} mm/s and amax={amax:.1f} mm/s^2"
        )


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
