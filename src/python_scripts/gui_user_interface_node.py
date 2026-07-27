#!/usr/bin/env python3
"""Operator-friendly Tkinter control panel for the delta robot."""

from __future__ import annotations

import math
import signal
import sys
import time
import tkinter as tk
from tkinter import messagebox, ttk

from ros_bootstrap import prepare_ros_python_environment
from ui_config import (
    DEFAULT_AMAX,
    DEFAULT_VMAX,
    HOME_TCP_MM,
    JOINT_STATE_TIMEOUT_SEC,
    MAX_Z_MM,
    MIN_Z_MM,
    SHAPES_BY_KEY,
    shape_message_type,
)

prepare_ros_python_environment(__file__)

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from my_delta_robot.msg import LinearSpeedXYZ, Posicionxyz, VmaxAmax  # noqa: E402


class RobotControlNode(Node):
    """ROS interface and small state cache used by the Tkinter application."""

    def __init__(self) -> None:
        super().__init__("gui_user_interface_node")
        self.motion_limits_pub = self.create_publisher(VmaxAmax, "set_vmax_amax", 10)
        self.shape_pub = self.create_publisher(Posicionxyz, "set_current_point", 10)
        self.segment_pub = self.create_publisher(LinearSpeedXYZ, "input_ls_final", 10)
        self.create_subscription(JointState, "joint_states", self._on_joint_state, 10)
        self.create_subscription(String, "status_delta", self._on_motion_status, 10)
        self.create_subscription(String, "drawing_status", self._on_drawing_status, 10)

        self.tcp_mm = HOME_TCP_MM
        self.has_joint_state = False
        self.last_joint_update = 0.0
        self.active_command: str | None = None
        self.pending_manual_target: tuple[float, float, float] | None = None
        self.events: list[tuple[str, str]] = []

    def _on_joint_state(self, msg: JointState) -> None:
        try:
            indices = [msg.name.index(name) for name in ("act_x", "act_y", "act_z")]
            self.tcp_mm = tuple(float(msg.position[index]) * 1000.0 for index in indices)
        except (ValueError, IndexError):
            return
        self.has_joint_state = True
        self.last_joint_update = time.monotonic()

    def _on_motion_status(self, msg: String) -> None:
        is_done = msg.data.startswith("DONE")
        is_failed = msg.data.startswith("FAILED:")
        if not is_done and not is_failed:
            self.events.append(("warning", f"Ignored motion status: {msg.data}"))
            return

        if self.active_command == "manual":
            if is_done and self.pending_manual_target is not None:
                self._publish_draw_current_point(self.pending_manual_target)
            self.reset_command_tracking()
            level = "error" if is_failed else "success"
            self.events.append((level, msg.data))
        elif is_failed:
            self.events.append(("error", msg.data))

    @staticmethod
    def _drawing_status_shape(status: str) -> str:
        detail = status.partition(":")[2].strip()
        return detail.split(" - ", maxsplit=1)[0]

    def _on_drawing_status(self, msg: String) -> None:
        is_started = msg.data.startswith("STARTED:")
        is_done = msg.data.startswith("DONE:")
        is_failed = msg.data.startswith("FAILED:")
        if not is_started and not is_done and not is_failed:
            self.events.append(("warning", f"Ignored drawing status: {msg.data}"))
            return

        status_shape = self._drawing_status_shape(msg.data)
        if is_started and self.active_command is None:
            self.active_command = "shape:external"
        elif (is_done or is_failed) and (
            self.active_command or ""
        ).startswith("shape:"):
            expected_shape = self.active_command.partition(":")[2]
            if expected_shape != "external" and status_shape != expected_shape:
                self.events.append(
                    (
                        "warning",
                        f"Ignored {status_shape} status while waiting for "
                        f"{expected_shape}",
                    )
                )
                return
            self.reset_command_tracking()
        level = "info" if is_started else "error" if is_failed else "success"
        self.events.append((level, msg.data))

    def connection_state(self) -> tuple[bool, str]:
        missing = []
        if self.motion_limits_pub.get_subscription_count() == 0:
            missing.append("main_node")
        if self.shape_pub.get_subscription_count() == 0:
            missing.append("draw_node")
        if self.segment_pub.get_subscription_count() == 0 and "main_node" not in missing:
            missing.append("main_node")
        if missing:
            return False, "Waiting for " + ", ".join(dict.fromkeys(missing))
        return True, "ROS connected"

    def joint_state_age(self) -> float | None:
        if not self.has_joint_state:
            return None
        return max(0.0, time.monotonic() - self.last_joint_update)

    def has_fresh_joint_state(
        self, timeout_sec: float = JOINT_STATE_TIMEOUT_SEC
    ) -> bool:
        age = self.joint_state_age()
        return age is not None and age <= timeout_sec

    def joint_state_description(self) -> str:
        age = self.joint_state_age()
        if age is None:
            return "waiting for TCP position"
        if age > JOINT_STATE_TIMEOUT_SEC:
            return f"TCP position stale ({age:.1f}s)"
        return "TCP live"

    def begin_command(
        self,
        command: str,
        manual_target: tuple[float, float, float] | None = None,
    ) -> None:
        self.active_command = command
        self.pending_manual_target = manual_target

    def reset_command_tracking(self) -> None:
        """Clear only this UI's bookkeeping; this does not cancel robot motion."""
        self.active_command = None
        self.pending_manual_target = None

    def publish_motion_limits(self, vmax: float, amax: float) -> None:
        msg = VmaxAmax()
        msg.vmax = vmax
        msg.amax = amax
        self.motion_limits_pub.publish(msg)

    def publish_shape(self, shape_key: str) -> None:
        shape = SHAPES_BY_KEY[shape_key]
        msg = Posicionxyz()
        msg.type = shape_message_type(shape, Posicionxyz)
        self.shape_pub.publish(msg)
        self.events.append(("info", f"Requested {shape.label.lower()}"))

    def _publish_draw_current_point(
        self, position_mm: tuple[float, float, float]
    ) -> None:
        """Keep draw_node's path origin aligned after a direct GUI move."""
        msg = Posicionxyz()
        msg.x0, msg.y0, msg.z0 = position_mm
        msg.type = Posicionxyz.SET_CURRENT_POINT
        self.shape_pub.publish(msg)

    def publish_move(self, target_mm: tuple[float, float, float]) -> None:
        msg = LinearSpeedXYZ()
        msg.xo, msg.yo, msg.zo = self.tcp_mm
        msg.xf, msg.yf, msg.zf = target_mm
        msg.gripper = 0
        self.segment_pub.publish(msg)
        self.events.append(
            (
                "info",
                f"Move to X={target_mm[0]:.1f}, Y={target_mm[1]:.1f}, "
                f"Z={target_mm[2]:.1f} mm",
            )
        )

    def drain_events(self) -> list[tuple[str, str]]:
        events, self.events = self.events, []
        return events


class RobotControlApp(ttk.Frame):
    """Control panel for shape drawing and direct Cartesian positioning."""

    def __init__(self, root: tk.Tk, node: RobotControlNode) -> None:
        super().__init__(root, padding=18)
        self.root = root
        self.node = node
        self.vmax_var = tk.StringVar(value=DEFAULT_VMAX)
        self.amax_var = tk.StringVar(value=DEFAULT_AMAX)
        self.connection_var = tk.StringVar(value="Connecting to ROS...")
        self.activity_var = tk.StringVar(value="Ready")
        self.position_vars = [tk.StringVar(value="--.-") for _ in range(3)]
        self.target_vars = [tk.StringVar(value=f"{value:.1f}") for value in HOME_TCP_MM]
        self.jog_step_var = tk.StringVar(value="5")
        self.command_buttons: list[ttk.Button] = []
        self.manual_buttons: list[ttk.Button] = []

        self.grid(row=0, column=0, sticky="nsew")
        root.columnconfigure(0, weight=1)
        root.rowconfigure(0, weight=1)
        self.columnconfigure(0, weight=1)
        self.rowconfigure(2, weight=1)

        self._configure_styles()
        self._build_header()
        self._build_limits()
        self._build_control_tabs()
        self._build_activity_log()
        self._schedule_ros_spin()
        self._refresh_ui()

    def _configure_styles(self) -> None:
        style = ttk.Style(self)
        style.configure("Title.TLabel", font=("TkDefaultFont", 18, "bold"))
        style.configure("Section.TLabel", font=("TkDefaultFont", 11, "bold"))
        style.configure("Position.TLabel", font=("TkFixedFont", 14, "bold"))
        style.configure("Connected.TLabel", foreground="#16803a")
        style.configure("Disconnected.TLabel", foreground="#b54708")
        style.configure("Primary.TButton", font=("TkDefaultFont", 11, "bold"), padding=10)

    def _build_header(self) -> None:
        header = ttk.Frame(self)
        header.grid(row=0, column=0, sticky="ew", pady=(0, 12))
        header.columnconfigure(0, weight=1)
        ttk.Label(header, text="Delta Robot Control", style="Title.TLabel").grid(
            row=0, column=0, sticky="w"
        )
        self.connection_label = ttk.Label(
            header, textvariable=self.connection_var, style="Disconnected.TLabel"
        )
        self.connection_label.grid(row=0, column=1, sticky="e")
        ttk.Label(
            header,
            text="Simulation control • commands execute immediately",
            foreground="#666666",
        ).grid(row=1, column=0, columnspan=2, sticky="w", pady=(3, 0))

    def _build_limits(self) -> None:
        frame = ttk.LabelFrame(self, text="Motion Limits", padding=10)
        frame.grid(row=1, column=0, sticky="ew", pady=(0, 12))
        for column in (1, 3):
            frame.columnconfigure(column, weight=1)

        ttk.Label(frame, text="Velocity (mm/s)").grid(row=0, column=0, sticky="w")
        ttk.Entry(frame, textvariable=self.vmax_var, width=14).grid(
            row=0, column=1, sticky="ew", padx=(8, 18)
        )
        ttk.Label(frame, text="Acceleration (mm/s²)").grid(row=0, column=2, sticky="w")
        ttk.Entry(frame, textvariable=self.amax_var, width=14).grid(
            row=0, column=3, sticky="ew", padx=(8, 18)
        )
        ttk.Button(frame, text="Apply", command=self._apply_limits).grid(row=0, column=4)

        presets = ttk.Frame(frame)
        presets.grid(row=1, column=0, columnspan=5, sticky="w", pady=(8, 0))
        ttk.Label(presets, text="Presets:").pack(side="left", padx=(0, 6))
        for label, velocity, acceleration in (
            ("Gentle", 500.0, 50.0),
            ("Normal", 2000.0, 100.0),
            ("Fast", 5000.0, 250.0),
        ):
            ttk.Button(
                presets,
                text=label,
                command=lambda v=velocity, a=acceleration: self._set_limit_preset(v, a),
            ).pack(side="left", padx=3)

    def _build_control_tabs(self) -> None:
        notebook = ttk.Notebook(self)
        notebook.grid(row=2, column=0, sticky="nsew")
        drawing_tab = ttk.Frame(notebook, padding=14)
        manual_tab = ttk.Frame(notebook, padding=14)
        notebook.add(drawing_tab, text="Draw Shapes")
        notebook.add(manual_tab, text="Manual Move / Jog")
        self._build_drawing_tab(drawing_tab)
        self._build_manual_tab(manual_tab)

    def _build_drawing_tab(self, tab: ttk.Frame) -> None:
        tab.columnconfigure((0, 1, 2), weight=1)
        ttk.Label(
            tab,
            text=(
                "Choose a shape. The robot approaches the drawing plane, draws, "
                "then returns home."
            ),
            wraplength=700,
        ).grid(row=0, column=0, columnspan=3, sticky="w", pady=(0, 14))

        for column, (key, shape) in enumerate(SHAPES_BY_KEY.items()):
            card = ttk.LabelFrame(tab, text=shape.label, padding=12)
            card.grid(row=1, column=column, sticky="nsew", padx=6)
            card.columnconfigure(0, weight=1)
            ttk.Label(
                card, text=shape.description, wraplength=180, anchor="center"
            ).grid(
                row=0, column=0, sticky="ew", pady=(0, 12)
            )
            button = ttk.Button(
                card,
                text=f"Draw {shape.label}",
                style="Primary.TButton",
                command=lambda shape=key: self._start_shape(shape),
            )
            button.grid(row=1, column=0, sticky="ew")
            self.command_buttons.append(button)

    def _build_manual_tab(self, tab: ttk.Frame) -> None:
        tab.columnconfigure(0, weight=1)
        tab.columnconfigure(1, weight=1)

        position = ttk.LabelFrame(tab, text="Live TCP Position", padding=12)
        position.grid(row=0, column=0, sticky="nsew", padx=(0, 6), pady=(0, 10))
        for column, axis in enumerate("XYZ"):
            position.columnconfigure(column, weight=1)
            ttk.Label(position, text=f"{axis} (mm)", anchor="center").grid(
                row=0, column=column, sticky="ew"
            )
            ttk.Label(
                position,
                textvariable=self.position_vars[column],
                style="Position.TLabel",
                anchor="center",
            ).grid(row=1, column=column, sticky="ew", pady=(4, 0))
        copy_button = ttk.Button(
            position, text="Copy to Target", command=self._copy_current_to_target
        )
        copy_button.grid(
            row=2, column=0, columnspan=3, pady=(10, 0), sticky="ew"
        )

        target = ttk.LabelFrame(tab, text="Move to Target", padding=12)
        target.grid(row=0, column=1, sticky="nsew", padx=(6, 0), pady=(0, 10))
        for row, axis in enumerate("XYZ"):
            ttk.Label(target, text=f"{axis} (mm)").grid(row=row, column=0, sticky="w", pady=3)
            ttk.Entry(target, textvariable=self.target_vars[row], width=14).grid(
                row=row, column=1, sticky="ew", padx=(8, 0), pady=3
            )
        target.columnconfigure(1, weight=1)
        move_button = ttk.Button(
            target, text="Move to Target", style="Primary.TButton", command=self._move_to_target
        )
        move_button.grid(row=3, column=0, columnspan=2, sticky="ew", pady=(10, 4))
        home_button = ttk.Button(target, text="Return Home", command=self._return_home)
        home_button.grid(row=4, column=0, columnspan=2, sticky="ew")
        self.command_buttons.extend((move_button, home_button))
        self.manual_buttons.extend((copy_button, move_button, home_button))

        jog = ttk.LabelFrame(tab, text="Incremental Jog", padding=12)
        jog.grid(row=1, column=0, columnspan=2, sticky="ew")
        ttk.Label(jog, text="Step (mm)").grid(row=0, column=0, sticky="w")
        ttk.Combobox(
            jog,
            textvariable=self.jog_step_var,
            values=("1", "2", "5", "10", "20"),
            state="readonly",
            width=7,
        ).grid(row=0, column=1, sticky="w", padx=(8, 20))
        ttk.Label(jog, text=f"Z workspace: {MIN_Z_MM:g} to {MAX_Z_MM:g} mm").grid(
            row=0, column=2, columnspan=4, sticky="w"
        )

        controls = (
            ("X −", 0, -1), ("X +", 0, 1),
            ("Y −", 1, -1), ("Y +", 1, 1),
            ("Z Down", 2, -1), ("Z Up", 2, 1),
        )
        for column, (label, axis, direction) in enumerate(controls):
            button = ttk.Button(
                jog,
                text=label,
                command=lambda a=axis, d=direction: self._jog(a, d),
            )
            button.grid(row=1, column=column, sticky="ew", padx=3, pady=(10, 0))
            jog.columnconfigure(column, weight=1)
            self.command_buttons.append(button)
            self.manual_buttons.append(button)

    def _build_activity_log(self) -> None:
        frame = ttk.LabelFrame(self, text="Activity", padding=8)
        frame.grid(row=3, column=0, sticky="ew", pady=(12, 0))
        frame.columnconfigure(0, weight=1)
        self.activity_text = tk.Text(frame, height=5, wrap="word", state="disabled")
        scrollbar = ttk.Scrollbar(frame, orient="vertical", command=self.activity_text.yview)
        self.activity_text.configure(yscrollcommand=scrollbar.set)
        self.activity_text.grid(row=0, column=0, sticky="ew")
        scrollbar.grid(row=0, column=1, sticky="ns")
        ttk.Label(frame, textvariable=self.activity_var, anchor="w").grid(
            row=1, column=0, sticky="ew", pady=(6, 0)
        )
        self.recovery_button = ttk.Button(
            frame, text="Recover UI Lock", command=self._recover_ui_lock
        )
        self.recovery_button.grid(row=1, column=1, sticky="e", padx=(8, 0), pady=(6, 0))
        self.recovery_button.configure(state="disabled")
        self._append_log("info", "Control panel ready")

    def _schedule_ros_spin(self) -> None:
        if rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.0)
            self.root.after(20, self._schedule_ros_spin)

    def _refresh_ui(self) -> None:
        connected, connection_text = self.node.connection_state()
        position_fresh = self.node.has_fresh_joint_state()
        connection_details = (
            f"{connection_text} • {self.node.joint_state_description()}"
            if connected
            else connection_text
        )
        self.connection_var.set(("● " if connected else "○ ") + connection_details)
        self.connection_label.configure(
            style="Connected.TLabel" if connected else "Disconnected.TLabel"
        )

        if position_fresh:
            for variable, value in zip(self.position_vars, self.node.tcp_mm):
                variable.set(f"{value:8.2f}")
        else:
            stale_text = "WAIT" if not self.node.has_joint_state else "STALE"
            for variable in self.position_vars:
                variable.set(stale_text)

        for level, message in self.node.drain_events():
            self._append_log(level, message)
            self.activity_var.set(message)

        busy = self.node.active_command is not None
        for button in self.command_buttons:
            button.configure(state="disabled" if busy else "normal")
        for button in self.manual_buttons:
            button.configure(
                state="normal" if position_fresh and not busy else "disabled"
            )
        self.recovery_button.configure(state="normal" if busy else "disabled")
        if busy and not self.activity_var.get().startswith("Busy"):
            self.activity_var.set("Busy — waiting for motion completion")

        self.root.after(100, self._refresh_ui)

    def _append_log(self, level: str, message: str) -> None:
        timestamp = time.strftime("%H:%M:%S")
        prefix = {
            "success": "OK",
            "error": "ERROR",
            "warning": "WARN",
            "info": "INFO",
        }.get(level, "INFO")
        self.activity_text.configure(state="normal")
        self.activity_text.insert("end", f"[{timestamp}] {prefix}: {message}\n")
        self.activity_text.see("end")
        self.activity_text.configure(state="disabled")

    def _read_limits(self) -> tuple[float, float] | None:
        try:
            vmax = float(self.vmax_var.get())
            amax = float(self.amax_var.get())
        except ValueError:
            messagebox.showerror(
                "Invalid motion limits", "Velocity and acceleration must be numbers."
            )
            return None
        if (
            not math.isfinite(vmax)
            or not math.isfinite(amax)
            or vmax <= 0.0
            or amax <= 0.0
        ):
            messagebox.showerror(
                "Invalid motion limits",
                "Velocity and acceleration must be finite and positive.",
            )
            return None
        return vmax, amax

    def _apply_limits(self, announce: bool = True) -> bool:
        limits = self._read_limits()
        if limits is None:
            return False
        if self.node.motion_limits_pub.get_subscription_count() == 0:
            messagebox.showerror(
                "main_node unavailable",
                "Motion limits were not sent because main_node is not connected.",
            )
            return False
        self.node.publish_motion_limits(*limits)
        if announce:
            message = f"Limits applied: {limits[0]:g} mm/s, {limits[1]:g} mm/s²"
            self._append_log("success", message)
            self.activity_var.set(message)
        return True

    def _set_limit_preset(self, velocity: float, acceleration: float) -> None:
        self.vmax_var.set(f"{velocity:g}")
        self.amax_var.set(f"{acceleration:g}")
        self._apply_limits()

    def _confirm_ready(self, require_fresh_position: bool = False) -> bool:
        if self.node.active_command is not None:
            messagebox.showwarning("Robot busy", "Wait for the current command to finish.")
            return False
        connected, details = self.node.connection_state()
        if not connected:
            messagebox.showerror(
                "ROS nodes not ready",
                f"{details}. Start the required nodes before sending a command.",
            )
            return False
        if require_fresh_position and not self.node.has_fresh_joint_state():
            messagebox.showwarning(
                "TCP position unavailable",
                (
                    f"Cannot move because the {self.node.joint_state_description()}. "
                    "Wait for fresh /joint_states data."
                ),
            )
            return False
        return True

    def _start_shape(self, shape_key: str) -> None:
        if not self._confirm_ready() or not self._apply_limits(announce=False):
            return
        shape = SHAPES_BY_KEY[shape_key]
        self.node.begin_command(f"shape:{shape_key}")
        self.activity_var.set(f"Starting {shape.label.lower()}...")
        self.root.after(150, lambda: self.node.publish_shape(shape_key))

    def _target_values(self) -> tuple[float, float, float] | None:
        try:
            values = tuple(float(variable.get()) for variable in self.target_vars)
        except ValueError:
            messagebox.showerror("Invalid target", "X, Y, and Z must be numbers.")
            return None
        if not all(math.isfinite(value) for value in values):
            messagebox.showerror("Invalid target", "Target coordinates must be finite.")
            return None
        if not MIN_Z_MM <= values[2] <= MAX_Z_MM:
            messagebox.showerror(
                "Target outside Z workspace",
                f"Z must be between {MIN_Z_MM:g} and {MAX_Z_MM:g} mm.",
            )
            return None
        return values

    def _dispatch_move(self, target: tuple[float, float, float]) -> None:
        self.node.publish_move(target)

    def _move_to_target(self) -> None:
        target = self._target_values()
        if (
            target is None
            or not self._confirm_ready(require_fresh_position=True)
            or not self._apply_limits(announce=False)
        ):
            return
        if all(
            abs(current - desired) < 1e-6
            for current, desired in zip(self.node.tcp_mm, target)
        ):
            messagebox.showinfo("Already at target", "The robot is already at this position.")
            return
        self.node.begin_command("manual", target)
        self.activity_var.set("Sending Cartesian move...")
        self.root.after(150, lambda: self._dispatch_move(target))

    def _copy_current_to_target(self) -> None:
        if not self.node.has_fresh_joint_state():
            messagebox.showwarning(
                "Position unavailable",
                (
                    f"Cannot copy because the {self.node.joint_state_description()}. "
                    "Wait for fresh /joint_states data."
                ),
            )
            return
        for variable, value in zip(self.target_vars, self.node.tcp_mm):
            variable.set(f"{value:.2f}")

    def _return_home(self) -> None:
        for variable, value in zip(self.target_vars, HOME_TCP_MM):
            variable.set(f"{value:.1f}")
        self._move_to_target()

    def _jog(self, axis: int, direction: int) -> None:
        if not self.node.has_fresh_joint_state():
            messagebox.showwarning(
                "Position unavailable",
                (
                    f"Cannot jog because the {self.node.joint_state_description()}. "
                    "Wait for fresh /joint_states data."
                ),
            )
            return
        step = float(self.jog_step_var.get())
        target = list(self.node.tcp_mm)
        target[axis] += direction * step
        for variable, value in zip(self.target_vars, target):
            variable.set(f"{value:.2f}")
        self._move_to_target()

    def _recover_ui_lock(self) -> None:
        if self.node.active_command is None:
            return
        confirmed = messagebox.askyesno(
            "Recover local UI lock",
            (
                "This only clears the control panel's local busy state. It does not "
                "stop or cancel robot motion.\n\nVerify main_node and draw_node are "
                "idle before continuing. Clear the local UI lock?"
            ),
        )
        if not confirmed:
            return
        previous_command = self.node.active_command
        self.node.reset_command_tracking()
        message = f"Cleared local UI lock for {previous_command}; no robot cancel was sent"
        self._append_log("warning", message)
        self.activity_var.set(message)


def main() -> int:
    rclpy.init()
    node = RobotControlNode()
    root = tk.Tk()
    root.title("Delta Robot Control")
    root.geometry("820x700")
    root.minsize(760, 640)
    RobotControlApp(root, node)
    root.protocol("WM_DELETE_WINDOW", root.destroy)

    def close_from_signal(_signum: int, _frame: object) -> None:
        root.after_idle(root.destroy)

    signal.signal(signal.SIGINT, close_from_signal)
    signal.signal(signal.SIGTERM, close_from_signal)

    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
