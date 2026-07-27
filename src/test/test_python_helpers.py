"""Unit tests for ROS-independent Python UI helpers."""

from __future__ import annotations

import pathlib
import sys
import unittest


SCRIPT_DIR = pathlib.Path(__file__).resolve().parents[1] / "python_scripts"
sys.path.insert(0, str(SCRIPT_DIR))

from ros_bootstrap import _resolve_python_from_shebang  # noqa: E402
from ui_config import SHAPES_BY_KEY, shape_message_type  # noqa: E402


class FakePositionMessage:
    """Minimal stand-in for generated Posicionxyz constants."""

    DRAW_RECTANGLE = 6
    DRAW_TRIANGLE = 7
    DRAW_CIRCLE = 8


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
    def test_shapes_resolve_generated_message_constants(self) -> None:
        resolved = {
            key: shape_message_type(shape, FakePositionMessage)
            for key, shape in SHAPES_BY_KEY.items()
        }
        self.assertEqual(
            resolved,
            {"rectangle": 6, "triangle": 7, "circle": 8},
        )


if __name__ == "__main__":
    unittest.main()
