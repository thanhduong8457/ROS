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
    DEFAULT_JOG_SPEED_MM_S,
    DEFAULT_VMAX,
    HOME_TCP_MM,
    JOG_DIRECTIONS,
    JOG_HEARTBEAT_PERIOD_MS,
    JOG_SPEED_OPTIONS_MM_S,
    JOINT_STATE_TIMEOUT_SEC,
    MAX_Z_MM,
    MIN_UI_VIEWPORT_SIZE,
    MIN_Z_MM,
    SHAPES_BY_KEY,
    fit_ui_window_size,
    jog_message_command,
    shape_message_type,
)

prepare_ros_python_environment(__file__)

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from my_delta_robot.msg import (  # noqa: E402
    CartesianJog,
    LinearSpeedXYZ,
    Posicionxyz,
    VmaxAmax,
)


class RobotControlNode(Node):
    """ROS interface and small state cache used by the Tkinter application."""

    def __init__(self) -> None:
        super().__init__("gui_user_interface_node")
        self.motion_limits_pub = self.create_publisher(VmaxAmax, "set_vmax_amax", 10)
        self.shape_pub = self.create_publisher(Posicionxyz, "set_current_point", 10)
        self.segment_pub = self.create_publisher(LinearSpeedXYZ, "input_ls_final", 10)
        self.jog_pub = self.create_publisher(
            CartesianJog, "input_cartesian_jog", 10
        )
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
        elif (self.active_command or "").startswith("jog:"):
            if is_done:
                self._publish_draw_current_point(self.tcp_mm)
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
        if self.jog_pub.get_subscription_count() == 0 and "main_node" not in missing:
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

    def publish_jog(self, direction_key: str, speed_mm_s: float) -> None:
        direction = JOG_DIRECTIONS[direction_key]
        msg = CartesianJog()
        msg.command = jog_message_command(direction, CartesianJog)
        msg.speed_mm_s = speed_mm_s
        self.jog_pub.publish(msg)

    def publish_jog_stop(self) -> None:
        msg = CartesianJog()
        msg.command = CartesianJog.STOP
        msg.speed_mm_s = 0.0
        self.jog_pub.publish(msg)

    def drain_events(self) -> list[tuple[str, str]]:
        events, self.events = self.events, []
        return events


class AutoHideScrollbar(ttk.Scrollbar):
    """Scrollbar that stays out of the layout until its canvas can scroll."""

    def set(self, first: str, last: str) -> None:
        if float(first) <= 0.0 and float(last) >= 1.0:
            self.grid_remove()
        else:
            self.grid()
        super().set(first, last)


class ScrollableControlArea(ttk.Frame):
    """Viewport that keeps the complete control panel reachable on small screens."""

    def __init__(self, parent: tk.Tk, node: RobotControlNode) -> None:
        super().__init__(parent)
        self.grid(row=0, column=0, sticky="nsew")
        self.columnconfigure(0, weight=1)
        self.rowconfigure(0, weight=1)

        self.canvas = tk.Canvas(
            self,
            highlightthickness=0,
            background=parent.cget("background"),
        )
        vertical = AutoHideScrollbar(
            self, orient="vertical", command=self.canvas.yview
        )
        horizontal = AutoHideScrollbar(
            self, orient="horizontal", command=self.canvas.xview
        )
        self.canvas.grid(row=0, column=0, sticky="nsew")
        vertical.grid(row=0, column=1, sticky="ns")
        horizontal.grid(row=1, column=0, sticky="ew")
        self.canvas.configure(
            yscrollcommand=vertical.set,
            xscrollcommand=horizontal.set,
        )

        self.app = RobotControlApp(self.canvas, node)
        self.content_window = self.canvas.create_window(
            (0, 0), window=self.app, anchor="nw"
        )
        self.app.bind("<Configure>", self._update_scroll_region)
        self.canvas.bind("<Configure>", self._resize_content)
        parent.bind("<MouseWheel>", self._scroll_vertical, add="+")
        parent.bind("<Button-4>", self._scroll_vertical, add="+")
        parent.bind("<Button-5>", self._scroll_vertical, add="+")
        parent.bind("<Shift-MouseWheel>", self._scroll_horizontal, add="+")

    def _update_scroll_region(self, _event: tk.Event) -> None:
        bounds = self.canvas.bbox(self.content_window)
        if bounds is not None:
            self.canvas.configure(scrollregion=bounds)

    def _resize_content(self, event: tk.Event) -> None:
        content_width = max(event.width, self.app.winfo_reqwidth())
        self.canvas.itemconfigure(self.content_window, width=content_width)

    @staticmethod
    def _wheel_units(event: tk.Event) -> int:
        if getattr(event, "num", None) == 4:
            return -1
        if getattr(event, "num", None) == 5:
            return 1
        return -1 if getattr(event, "delta", 0) > 0 else 1

    def _scroll_vertical(self, event: tk.Event) -> str | None:
        first, last = self.canvas.yview()
        if first <= 0.0 and last >= 1.0:
            return None
        self.canvas.yview_scroll(self._wheel_units(event), "units")
        return "break"

    def _scroll_horizontal(self, event: tk.Event) -> str | None:
        first, last = self.canvas.xview()
        if first <= 0.0 and last >= 1.0:
            return None
        self.canvas.xview_scroll(self._wheel_units(event), "units")
        return "break"


class RobotControlApp(ttk.Frame):
    """Control panel for shape drawing and direct Cartesian positioning."""

    def __init__(self, parent: tk.Misc, node: RobotControlNode) -> None:
        super().__init__(parent, padding=18)
        self.root = parent.winfo_toplevel()
        self.node = node
        self.vmax_var = tk.StringVar(value=DEFAULT_VMAX)
        self.amax_var = tk.StringVar(value=DEFAULT_AMAX)
        self.connection_var = tk.StringVar(value="Connecting to ROS...")
        self.activity_var = tk.StringVar(value="Ready")
        self.position_vars = [tk.StringVar(value="--.-") for _ in range(3)]
        self.target_vars = [tk.StringVar(value=f"{value:.1f}") for value in HOME_TCP_MM]
        self.jog_speed_var = tk.StringVar(value=DEFAULT_JOG_SPEED_MM_S)
        self.active_jog_direction: str | None = None
        self.command_buttons: list[ttk.Button] = []
        self.manual_buttons: list[ttk.Button] = []
        self.jog_buttons: dict[str, ttk.Button] = {}

        self.columnconfigure(0, weight=1)
        self.rowconfigure(2, weight=1)

        self._configure_styles()
        self._build_header()
        self._build_limits()
        self._build_control_tabs()
        self._build_activity_log()
        self.root.bind_all(
            "<ButtonRelease-1>", self._on_pointer_release, add="+"
        )
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
        style.configure(
            "Joystick.TButton",
            font=("TkDefaultFont", 10, "bold"),
            padding=(12, 9),
        )

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

        jog = ttk.LabelFrame(tab, text="Cartesian Joystick", padding=12)
        jog.grid(row=1, column=0, columnspan=2, sticky="ew")
        ttk.Label(jog, text="Jog speed (mm/s)").grid(
            row=0, column=0, sticky="w"
        )
        ttk.Combobox(
            jog,
            textvariable=self.jog_speed_var,
            values=JOG_SPEED_OPTIONS_MM_S,
            state="readonly",
            width=7,
        ).grid(row=0, column=1, sticky="w", padx=(8, 20))
        ttk.Label(
            jog,
            text=(
                "Press and hold to move; release to stop • "
                f"Z workspace: {MIN_Z_MM:g} to {MAX_Z_MM:g} mm"
            ),
        ).grid(
            row=0, column=2, sticky="w"
        )
        jog.columnconfigure(2, weight=1)

        xy_pad = ttk.LabelFrame(jog, text="Horizontal Plane (X / Y)", padding=8)
        xy_pad.grid(row=1, column=0, columnspan=2, sticky="nsew", pady=(10, 0))
        for column in range(3):
            xy_pad.columnconfigure(column, weight=1)
        xy_controls = (
            ("forward", 0, 1, "↑ Forward\n(+Y)"),
            ("left", 1, 0, "← Left\n(−X)"),
            ("right", 1, 2, "Right →\n(+X)"),
            ("back", 2, 1, "↓ Back\n(−Y)"),
        )
        for direction_key, row, column, label in xy_controls:
            button = ttk.Button(
                xy_pad,
                text=label,
                style="Joystick.TButton",
            )
            button.bind(
                "<ButtonPress-1>",
                lambda _event, key=direction_key: self._start_jog(key),
            )
            button.grid(row=row, column=column, sticky="nsew", padx=4, pady=4)
            self.jog_buttons[direction_key] = button
            self.command_buttons.append(button)
            self.manual_buttons.append(button)
        ttk.Label(
            xy_pad,
            text="TCP\nX / Y",
            anchor="center",
            justify="center",
            style="Section.TLabel",
        ).grid(row=1, column=1, sticky="nsew", padx=4, pady=4)

        z_pad = ttk.LabelFrame(jog, text="Vertical Axis (Z)", padding=8)
        z_pad.grid(row=1, column=2, sticky="nsew", padx=(12, 0), pady=(10, 0))
        z_pad.columnconfigure(0, weight=1)
        for row, direction_key, label in (
            (0, "up", "↑ Up\n(+Z)"),
            (2, "down", "↓ Down\n(−Z)"),
        ):
            button = ttk.Button(
                z_pad,
                text=label,
                style="Joystick.TButton",
            )
            button.bind(
                "<ButtonPress-1>",
                lambda _event, key=direction_key: self._start_jog(key),
            )
            button.grid(row=row, column=0, sticky="nsew", padx=4, pady=4)
            self.jog_buttons[direction_key] = button
            self.command_buttons.append(button)
            self.manual_buttons.append(button)
        ttk.Label(
            z_pad,
            text="End Effector\nHeight",
            anchor="center",
            justify="center",
            style="Section.TLabel",
        ).grid(row=1, column=0, sticky="nsew", padx=4, pady=4)

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

        if self.active_jog_direction is not None and not (
            self.node.active_command or ""
        ).startswith("jog:"):
            self.active_jog_direction = None

        busy = self.node.active_command is not None
        jogging = (self.node.active_command or "").startswith("jog:")
        for button in self.command_buttons:
            button.configure(state="disabled" if busy else "normal")
        for button in self.manual_buttons:
            button.configure(
                state="normal" if position_fresh and not busy else "disabled"
            )
        if jogging and self.active_jog_direction is not None:
            self.jog_buttons[self.active_jog_direction].configure(state="normal")
        self.recovery_button.configure(
            state="normal" if busy and not jogging else "disabled"
        )
        if busy and not jogging and not self.activity_var.get().startswith("Busy"):
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

    def _move_to_target(
        self, activity_message: str = "Sending Cartesian move..."
    ) -> None:
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
        self.activity_var.set(activity_message)
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

    def _read_jog_speed(self) -> float | None:
        try:
            speed = float(self.jog_speed_var.get())
        except ValueError:
            messagebox.showerror("Invalid jog speed", "Jog speed must be a number.")
            return None
        if not math.isfinite(speed) or speed <= 0.0:
            messagebox.showerror(
                "Invalid jog speed", "Jog speed must be finite and positive."
            )
            return None
        return speed

    def _start_jog(self, direction_key: str) -> None:
        if self.active_jog_direction is not None:
            return
        speed = self._read_jog_speed()
        if (
            speed is None
            or not self._confirm_ready(require_fresh_position=True)
            or not self._apply_limits(announce=False)
        ):
            return
        direction = JOG_DIRECTIONS[direction_key]
        self.active_jog_direction = direction_key
        self.node.begin_command(f"jog:{direction_key}")
        self.node.publish_jog(direction_key, speed)
        self.activity_var.set(
            f"Jogging {direction.label.lower()} ({direction.coordinate}) "
            "— release button to stop"
        )
        self.root.after(JOG_HEARTBEAT_PERIOD_MS, self._send_jog_heartbeat)

    def _send_jog_heartbeat(self) -> None:
        direction_key = self.active_jog_direction
        if direction_key is None:
            return
        if self.node.active_command != f"jog:{direction_key}":
            self.active_jog_direction = None
            return
        speed = self._read_jog_speed()
        if speed is None:
            self._stop_jog()
            return
        self.node.publish_jog(direction_key, speed)
        self.root.after(JOG_HEARTBEAT_PERIOD_MS, self._send_jog_heartbeat)

    def _on_pointer_release(self, _event: tk.Event) -> None:
        self._stop_jog()

    def _stop_jog(self) -> None:
        direction_key = self.active_jog_direction
        if direction_key is None:
            return
        self.active_jog_direction = None
        self.node.publish_jog_stop()
        for variable, value in zip(self.target_vars, self.node.tcp_mm):
            variable.set(f"{value:.2f}")
        direction = JOG_DIRECTIONS[direction_key]
        self.activity_var.set(
            f"Stopping {direction.label.lower()} jog at the current position..."
        )

    def stop_jog_before_shutdown(self) -> None:
        """Send an explicit stop; main_node's heartbeat timeout is the fallback."""
        self._stop_jog()

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
    screen_width = root.winfo_screenwidth()
    screen_height = root.winfo_screenheight()
    window_width, window_height = fit_ui_window_size(screen_width, screen_height)
    offset_x = max(0, (screen_width - window_width) // 2)
    offset_y = max(0, (screen_height - window_height) // 2)
    root.geometry(
        f"{window_width}x{window_height}+{offset_x}+{offset_y}"
    )
    root.minsize(
        min(MIN_UI_VIEWPORT_SIZE[0], window_width),
        min(MIN_UI_VIEWPORT_SIZE[1], window_height),
    )
    root.columnconfigure(0, weight=1)
    root.rowconfigure(0, weight=1)
    controls = ScrollableControlArea(root, node)
    app = controls.app

    def close_window() -> None:
        app.stop_jog_before_shutdown()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", close_window)

    def close_from_signal(_signum: int, _frame: object) -> None:
        root.after_idle(close_window)

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
