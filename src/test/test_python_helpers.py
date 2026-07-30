"""Unit tests for ROS-independent Python UI helpers."""

from __future__ import annotations

import pathlib
import sys
import unittest


SCRIPT_DIR = pathlib.Path(__file__).resolve().parents[1] / "python_scripts"
sys.path.insert(0, str(SCRIPT_DIR))

from ros_bootstrap import _resolve_python_from_shebang  # noqa: E402
from ui_config import (  # noqa: E402
    JOG_DIRECTIONS,
    SHAPES_BY_KEY,
    fit_ui_window_size,
    jog_message_command,
    shape_message_type,
)


class FakePositionMessage:
    """Minimal stand-in for generated Posicionxyz constants."""

    DRAW_RECTANGLE = 6
    DRAW_TRIANGLE = 7
    DRAW_CIRCLE = 8


class FakeJogMessage:
    """Minimal stand-in for generated CartesianJog constants."""

    STOP = 0
    FORWARD = 1
    BACK = 2
    LEFT = 3
    RIGHT = 4
    UP = 5
    DOWN = 6


class RosBootstrapTest(unittest.TestCase):
    def test_resolves_direct_python_shebang(self) -> None:
        self.assertEqual(
            _resolve_python_from_shebang(f"#!{sys.executable}"),
            sys.executable,
        )

    def test_resolves_env_python_shebang(self) -> None:
        resolved = _resolve_python_from_shebang("#!/usr/bin/env python3")
        self.assertIsNotNone(resolved)
        self.assertTrue(pathlib.Path(resolved).name.startswith("python"))

    def test_rejects_non_python_or_malformed_shebang(self) -> None:
        self.assertIsNone(_resolve_python_from_shebang("python3"))
        self.assertIsNone(_resolve_python_from_shebang("#!/bin/sh"))
        self.assertIsNone(_resolve_python_from_shebang("#!"))


class UiConfigTest(unittest.TestCase):
    def test_control_window_fits_smaller_screens(self) -> None:
        self.assertEqual(fit_ui_window_size(1920, 1080), (900, 760))
        self.assertEqual(fit_ui_window_size(800, 600), (720, 500))
        self.assertEqual(fit_ui_window_size(400, 300), (400, 300))

    def test_shapes_resolve_generated_message_constants(self) -> None:
        resolved = {
            key: shape_message_type(shape, FakePositionMessage)
            for key, shape in SHAPES_BY_KEY.items()
        }
        self.assertEqual(
            resolved,
            {"rectangle": 6, "triangle": 7, "circle": 8},
        )

    def test_cartesian_joystick_message_mapping(self) -> None:
        resolved = {
            key: jog_message_command(direction, FakeJogMessage)
            for key, direction in JOG_DIRECTIONS.items()
        }
        self.assertEqual(
            resolved,
            {
                "forward": 1,
                "back": 2,
                "left": 3,
                "right": 4,
                "up": 5,
                "down": 6,
            },
        )


if __name__ == "__main__":
    unittest.main()
