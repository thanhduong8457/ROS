"""Small runtime bootstrap shared by the executable ROS Python UIs.

This module deliberately has no import-time side effects. Executables call
``prepare_ros_python_environment`` before importing rclpy or generated messages.
"""

from __future__ import annotations

import ctypes
import os
import shlex
import shutil
import sys


PACKAGE_NAME = "my_delta_robot"
REEXEC_GUARD = "MY_DELTA_ROBOT_ROS_PYTHON_REEXEC"
TYPE_SUPPORT_LIBRARIES = (
    "libmy_delta_robot__rosidl_generator_c.dylib",
    "libmy_delta_robot__rosidl_typesupport_c.dylib",
    "libmy_delta_robot__rosidl_typesupport_fastrtps_c.dylib",
    "libmy_delta_robot__rosidl_typesupport_introspection_c.dylib",
    "libmy_delta_robot__rosidl_generator_py.dylib",
)


def get_ament_prefixes() -> list[str]:
    """Return non-empty prefixes from the active ament environment."""
    return [
        prefix
        for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep)
        if prefix
    ]


def ensure_ros_library_paths() -> None:
    """Restore workspace library paths stripped from macOS script launches."""
    library_dirs = [
        os.path.join(prefix, "lib")
        for prefix in get_ament_prefixes()
        if os.path.isdir(os.path.join(prefix, "lib"))
    ]
    for variable_name in ("DYLD_LIBRARY_PATH", "LD_LIBRARY_PATH"):
        current_paths = [
            path
            for path in os.environ.get(variable_name, "").split(os.pathsep)
            if path
        ]
        merged_paths = list(dict.fromkeys([*library_dirs, *current_paths]))
        if merged_paths:
            os.environ[variable_name] = os.pathsep.join(merged_paths)


def _resolve_python_from_shebang(shebang: str) -> str | None:
    """Resolve direct and conventional ``/usr/bin/env python`` shebangs."""
    if not shebang.startswith("#!"):
        return None
    try:
        command = shlex.split(shebang[2:].strip())
    except ValueError:
        return None
    if not command:
        return None

    executable = command[0]
    if os.path.basename(executable) == "env":
        arguments = command[1:]
        if arguments[:1] in (["-S"], ["--split-string"]):
            arguments = arguments[1:]
        if not arguments or arguments[0].startswith("-") or "=" in arguments[0]:
            return None
        executable = shutil.which(arguments[0]) or ""

    if not executable or not os.path.isabs(executable):
        executable = shutil.which(executable) or ""
    if (
        not executable
        or not os.path.isfile(executable)
        or not os.access(executable, os.X_OK)
        or not os.path.basename(executable).lower().startswith("python")
    ):
        return None
    return executable


def relaunch_with_ros_python(script_path: str) -> None:
    """Re-execute with ros2's concrete Python interpreter when it is resolvable."""
    if os.environ.get(REEXEC_GUARD) == "1":
        return

    ros2_path = shutil.which("ros2")
    if not ros2_path:
        return
    try:
        with open(ros2_path, "r", encoding="utf-8") as ros2_file:
            ros_python = _resolve_python_from_shebang(ros2_file.readline().strip())
    except OSError:
        return
    if not ros_python or os.path.realpath(ros_python) == os.path.realpath(sys.executable):
        return

    os.environ[REEXEC_GUARD] = "1"
    executable_script = os.path.abspath(script_path)
    try:
        os.execv(ros_python, [ros_python, executable_script, *sys.argv[1:]])
    except OSError:
        # Continue so the subsequent import reports the actual missing dependency.
        return


def ensure_workspace_log_dir() -> None:
    """Use the workspace log directory when ROS_LOG_DIR is not explicitly set."""
    if os.environ.get("ROS_LOG_DIR"):
        return
    for prefix in get_ament_prefixes():
        if os.path.basename(prefix) != PACKAGE_NAME:
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
    """Load generated package dylibs by absolute path on macOS."""
    if sys.platform != "darwin":
        return
    load_mode = getattr(ctypes, "RTLD_GLOBAL", 0)
    for prefix in get_ament_prefixes():
        for library_name in TYPE_SUPPORT_LIBRARIES:
            library_path = os.path.join(prefix, "lib", library_name)
            if not os.path.isfile(library_path):
                continue
            try:
                ctypes.CDLL(library_path, mode=load_mode)
            except OSError:
                continue


def prepare_ros_python_environment(script_path: str) -> None:
    """Prepare loader, interpreter, logging, and type support for a ROS UI."""
    ensure_ros_library_paths()
    relaunch_with_ros_python(script_path)
    ensure_workspace_log_dir()
    preload_ros_type_support_libraries()
