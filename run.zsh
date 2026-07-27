#!/usr/bin/env zsh
set -euo pipefail

SCRIPT_DIR="${0:A:h}"
SETUP_FILE="${SCRIPT_DIR}/install/setup.zsh"

if [[ ! -f "${SETUP_FILE}" ]]; then
  print -u2 "Workspace is not built yet."
  print -u2 "Run ./build.zsh, then try again."
  exit 1
fi

mode="${1:-control}"
if (( $# > 0 )); then
  shift
fi

# Generated colcon setup files may read optional variables without defaults.
set +u
source "${SETUP_FILE}"
set -u

case "${mode}" in
  control)
    exec ros2 launch my_delta_robot bringup.launch.py \
      gui:=true rviz:=true "$@"
    ;;
  display)
    exec ros2 launch my_delta_robot bringup.launch.py \
      gui:=false rviz:=true "$@"
    ;;
  headless)
    exec ros2 launch my_delta_robot bringup.launch.py \
      gui:=false rviz:=false "$@"
    ;;
  *)
    print -u2 "Unknown mode: ${mode}"
    print -u2 "Usage: ./run.zsh [control|display|headless] [launch arguments]"
    exit 2
    ;;
esac
