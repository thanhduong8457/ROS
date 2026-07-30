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
DEFAULT_JOG_SPEED_MM_S = "25"
JOG_SPEED_OPTIONS_MM_S = ("5", "10", "25", "50", "100")
JOG_HEARTBEAT_PERIOD_MS = 100

DRAWING_START_TIMEOUT_SEC = 5.0
DRAWING_COMPLETION_TIMEOUT_SEC = 120.0

PREFERRED_UI_SIZE = (900, 760)
MIN_UI_VIEWPORT_SIZE = (480, 360)
UI_SCREEN_MARGIN = (80, 100)


@dataclass(frozen=True)
class ShapeDefinition:
    """Operator-facing metadata and the matching Posicionxyz constant name."""

    key: str
    label: str
    message_constant: str
    description: str


@dataclass(frozen=True)
class JogDirection:
    """Operator-facing Cartesian direction and its coordinate-axis mapping."""

    key: str
    label: str
    message_constant: str
    coordinate: str


JOG_DIRECTIONS = {
    "forward": JogDirection("forward", "Forward", "FORWARD", "+Y"),
    "back": JogDirection("back", "Back", "BACK", "−Y"),
    "left": JogDirection("left", "Left", "LEFT", "−X"),
    "right": JogDirection("right", "Right", "RIGHT", "+X"),
    "up": JogDirection("up", "Up", "UP", "+Z"),
    "down": JogDirection("down", "Down", "DOWN", "−Z"),
}


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


def fit_ui_window_size(screen_width: int, screen_height: int) -> tuple[int, int]:
    """Fit the preferred control window inside a display with safe edge margins."""
    dimensions = []
    for screen, preferred, minimum, margin in zip(
        (screen_width, screen_height),
        PREFERRED_UI_SIZE,
        MIN_UI_VIEWPORT_SIZE,
        UI_SCREEN_MARGIN,
    ):
        screen = max(1, int(screen))
        available = max(1, screen - margin)
        dimensions.append(min(screen, preferred, max(minimum, available)))
    return dimensions[0], dimensions[1]


def shape_message_type(shape: ShapeDefinition, message_class: type) -> int:
    """Resolve a generated ROS message constant without duplicating its value."""
    return int(getattr(message_class, shape.message_constant))


def jog_message_command(direction: JogDirection, message_class: type) -> int:
    """Resolve a generated CartesianJog constant without duplicating its value."""
    return int(getattr(message_class, direction.message_constant))
