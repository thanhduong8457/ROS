#!/usr/bin/env zsh
set -euo pipefail

SCRIPT_DIR="${0:A:h}"
cd "${SCRIPT_DIR}"

if [[ -n "${CONDA_PREFIX:-}" ]]; then
  colcon build --symlink-install --packages-select my_delta_robot \
    --cmake-args -DPython3_FIND_VIRTUALENV=ONLY "$@"
else
  colcon build --symlink-install --packages-select my_delta_robot "$@"
fi

echo ""
echo "Build OK. Source the workspace:"
echo "  source install/setup.zsh"
