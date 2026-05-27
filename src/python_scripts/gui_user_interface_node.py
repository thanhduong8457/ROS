#!/usr/bin/env python3
"""Tkinter GUI for sending drawing commands to the delta robot."""

from __future__ import annotations

import ctypes
import os
import shutil
import sys


def relaunch_with_ros_python() -> None:
    """Use the Python interpreter that launched ros2 when the shell uses another python3."""
    ensure_ros_library_paths()

    if os.environ.get("MY_DELTA_GUI_UI_REEXEC") == "1":
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

    os.environ["MY_DELTA_GUI_UI_REEXEC"] = "1"
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
import tkinter as tk
from tkinter import messagebox, ttk

from my_delta_robot.msg import Posicionxyz, VmaxAmax


DEFAULT_VMAX = "5000.0"
DEFAULT_AMAX = "100.0"

SHAPES = {
    "triangle": ("Triangle", 7),
    "rectangle": ("Rectangle", 6),
    "circle": ("Circle", 8),
}


class DrawingCommandPublisher(Node):
    """ROS publisher wrapper used by the GUI."""

    def __init__(self) -> None:
        super().__init__("gui_user_interface_node")
        self.motion_limits_pub = self.create_publisher(VmaxAmax, "set_vmax_amax", 10)
        self.target_action_pub = self.create_publisher(Posicionxyz, "set_current_point", 10)

    def subscriber_warnings(self) -> list[str]:
        warnings: list[str] = []
        if self.motion_limits_pub.get_subscription_count() == 0:
            warnings.append("No subscriber on /set_vmax_amax. Start main_node to apply limits.")
        if self.target_action_pub.get_subscription_count() == 0:
            warnings.append("No subscriber on /set_current_point. Start draw_node to draw shapes.")
        return warnings

    def publish_motion_limits(self, vmax: float, amax: float) -> None:
        limits_msg = VmaxAmax()
        limits_msg.vmax = vmax
        limits_msg.amax = amax

        # Publish a few times so a just-started subscriber is less likely to miss the update.
        for _ in range(3):
            self.motion_limits_pub.publish(limits_msg)

    def publish_shape_command(self, shape_type: int) -> None:
        action_msg = Posicionxyz()
        action_msg.x0 = 0.0
        action_msg.y0 = 0.0
        action_msg.z0 = 0.0
        action_msg.type = shape_type
        self.target_action_pub.publish(action_msg)


class DrawingCommandApp(ttk.Frame):
    """Small Tkinter front end for the drawing command publisher."""

    def __init__(self, root: tk.Tk, node: DrawingCommandPublisher) -> None:
        super().__init__(root, padding=16)
        self.root = root
        self.node = node
        self.vmax_var = tk.StringVar(value=DEFAULT_VMAX)
        self.amax_var = tk.StringVar(value=DEFAULT_AMAX)
        self.shape_var = tk.StringVar(value="")
        self.status_var = tk.StringVar(value="Ready")

        self.grid(row=0, column=0, sticky="nsew")
        root.columnconfigure(0, weight=1)
        root.rowconfigure(0, weight=1)

        self._build_widgets()
        self._schedule_ros_spin()
        self._schedule_status_update()

    def _build_widgets(self) -> None:
        self.columnconfigure(1, weight=1)

        title = ttk.Label(self, text="Delta Robot Drawing Command", font=("", 16, "bold"))
        title.grid(row=0, column=0, columnspan=2, sticky="w", pady=(0, 16))

        ttk.Label(self, text="vmax (mm/s)").grid(row=1, column=0, sticky="w", padx=(0, 12), pady=6)
        vmax_entry = ttk.Entry(self, textvariable=self.vmax_var, width=18)
        vmax_entry.grid(row=1, column=1, sticky="ew", pady=6)

        ttk.Label(self, text="amax (mm/s^2)").grid(row=2, column=0, sticky="w", padx=(0, 12), pady=6)
        amax_entry = ttk.Entry(self, textvariable=self.amax_var, width=18)
        amax_entry.grid(row=2, column=1, sticky="ew", pady=6)

        shape_frame = ttk.LabelFrame(self, text="Drawing Action", padding=10)
        shape_frame.grid(row=3, column=0, columnspan=2, sticky="ew", pady=(14, 10))
        shape_frame.columnconfigure((0, 1, 2), weight=1)

        for column, (shape_key, (shape_label, _)) in enumerate(SHAPES.items()):
            ttk.Radiobutton(
                shape_frame,
                text=shape_label,
                value=shape_key,
                variable=self.shape_var,
            ).grid(row=0, column=column, sticky="w", padx=(0, 12))

        button_frame = ttk.Frame(self)
        button_frame.grid(row=4, column=0, columnspan=2, sticky="ew", pady=(8, 12))
        button_frame.columnconfigure(0, weight=1)
        button_frame.columnconfigure(1, weight=1)

        send_button = ttk.Button(button_frame, text="Start Drawing", command=self._on_send)
        send_button.grid(row=0, column=0, sticky="ew", padx=(0, 6))

        reset_button = ttk.Button(button_frame, text="Reset Form", command=self._on_reset)
        reset_button.grid(row=0, column=1, sticky="ew", padx=(6, 0))

        status_label = ttk.Label(self, textvariable=self.status_var, anchor="w")
        status_label.grid(row=5, column=0, columnspan=2, sticky="ew")

        vmax_entry.focus_set()

    def _schedule_ros_spin(self) -> None:
        if rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.0)
            self.root.after(50, self._schedule_ros_spin)

    def _schedule_status_update(self) -> None:
        warnings = self.node.subscriber_warnings()
        if warnings:
            self.status_var.set(warnings[0])
        elif self.status_var.get().startswith("No subscriber"):
            self.status_var.set("Connected to drawing command topics")

        self.root.after(1000, self._schedule_status_update)

    def _validate_inputs(self) -> tuple[float, float, str, int] | None:
        try:
            vmax = float(self.vmax_var.get())
        except ValueError:
            messagebox.showerror("Invalid vmax", "vmax must be numeric and positive.")
            return None

        try:
            amax = float(self.amax_var.get())
        except ValueError:
            messagebox.showerror("Invalid amax", "amax must be numeric and positive.")
            return None

        if vmax <= 0.0:
            messagebox.showerror("Invalid vmax", "vmax must be greater than 0.")
            return None

        if amax <= 0.0:
            messagebox.showerror("Invalid amax", "amax must be greater than 0.")
            return None

        shape_key = self.shape_var.get()
        if shape_key not in SHAPES:
            messagebox.showerror("Missing shape", "Select Triangle, Rectangle, or Circle.")
            return None

        shape_label, shape_type = SHAPES[shape_key]
        return vmax, amax, shape_label, shape_type

    def _on_send(self) -> None:
        validated = self._validate_inputs()
        if validated is None:
            return

        vmax, amax, shape_label, shape_type = validated
        warnings = self.node.subscriber_warnings()
        if warnings and not messagebox.askyesno(
            "ROS subscribers not detected",
            "\n".join(warnings) + "\n\nPublish the command anyway?",
        ):
            return

        self.node.publish_motion_limits(vmax, amax)
        self.root.after(150, lambda: self._publish_shape(shape_label, shape_type, vmax, amax))
        self.status_var.set(f"Sending {shape_label.lower()} command...")

    def _publish_shape(self, shape_label: str, shape_type: int, vmax: float, amax: float) -> None:
        self.node.publish_shape_command(shape_type)
        self.node.get_logger().info(
            f"Sent {shape_label.lower()} command with vmax={vmax:.1f} mm/s and "
            f"amax={amax:.1f} mm/s^2"
        )
        self.status_var.set(
            f"Sent {shape_label.lower()} command: vmax={vmax:g} mm/s, amax={amax:g} mm/s^2"
        )

    def _on_reset(self) -> None:
        self.vmax_var.set(DEFAULT_VMAX)
        self.amax_var.set(DEFAULT_AMAX)
        self.shape_var.set("")
        self.status_var.set("Ready")


def main() -> int:
    rclpy.init()
    node = DrawingCommandPublisher()

    root = tk.Tk()
    root.title("Delta Robot GUI")
    root.minsize(420, 260)
    DrawingCommandApp(root, node)

    def on_close() -> None:
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)

    try:
        root.mainloop()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(main())
