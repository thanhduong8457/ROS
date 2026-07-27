# Archived Experiments

This directory preserves prototypes from earlier versions of the project. None
of these files are built, installed, launched, or tested by the active ROS 2
package.

- `cpp/` contains the former image dispatcher, drawing node, and serial bridge.
- `python/` contains ROS 1 camera scripts, standalone OpenCV experiments, a
  tutorial node, and the redundant initial-pose publisher.
- `interfaces/` contains messages used only by those archived experiments.
- `launch/` contains the redundant initial-pose launcher.

The archive is reference material, not supported runtime code. Port a component
to ROS 2, add tests, and move it into the active package before using it.

Sample images used by the camera experiments remain in `../images/` so binary
assets keep their existing Git history.
