"""Pure configuration shared by the graphical and terminal robot UIs."""

from __future__ import annotations

from dataclasses import dataclass


DEFAULT_VMAX = 5000.0
DEFAULT_AMAX = 100.0
MIN_VMAX = 1.0
MIN_AMAX = 1.0

HOME_TCP_MM = (0.0, 0.0, -375.0)
MIN_Z_MM = -480.0
MAX_Z_MM = -375.0
JOINT_STATE_TIMEOUT_SEC = 1.0

DRAWING_START_TIMEOUT_SEC = 5.0
DRAWING_COMPLETION_TIMEOUT_SEC = 120.0


@dataclass(frozen=True)
class ShapeDefinition:
    """Operator-facing metadata and the matching Posicionxyz constant name."""

    key: str
    label: str
    message_constant: str
    description: str


SHAPES_BY_KEY = {
    "rectangle": ShapeDefinition(
        "rectangle", "Rectangle", "DRAW_RECTANGLE", "Four straight sides"
    ),
    "triangle": ShapeDefinition(
        "triangle", "Triangle", "DRAW_TRIANGLE", "Three straight sides"
    ),
    "circle": ShapeDefinition(
        "circle",
        "Smooth Circle",
        "DRAW_CIRCLE",
        "Continuous curvature-limited path",
    ),
}

TERMINAL_SHAPE_OPTIONS = {
    "1": SHAPES_BY_KEY["triangle"],
    "2": SHAPES_BY_KEY["rectangle"],
    "3": SHAPES_BY_KEY["circle"],
}


def shape_message_type(shape: ShapeDefinition, message_class: type) -> int:
    """Resolve a generated ROS message constant without duplicating its value."""
    return int(getattr(message_class, shape.message_constant))
