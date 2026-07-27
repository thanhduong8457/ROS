#!/usr/bin/env zsh
set -euo pipefail

SCRIPT_DIR="${0:A:h}"
cd "${SCRIPT_DIR}"

function usage() {
  print "Usage: ./build.zsh [--test] [colcon build arguments]"
  print ""
  print "  --test      Run package tests and print verbose results after building."
  print "  -h, --help  Show this help."
}

run_tests=false
python_cmake_arg_added=false
typeset -a build_args
ros_python_executable="$(command -v python3)"
python_cmake_arg="-DPython3_EXECUTABLE=${ros_python_executable}"

while (( $# > 0 )); do
  case "$1" in
    --test)
      run_tests=true
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --cmake-args)
      build_args+=("$1" "${python_cmake_arg}")
      python_cmake_arg_added=true
      ;;
    *)
      build_args+=("$1")
      if [[ "$1" == -DPython3_EXECUTABLE=* ]]; then
        ros_python_executable="${1#*=}"
      fi
      ;;
  esac
  shift
done

if [[ "${python_cmake_arg_added}" == false ]]; then
  build_args+=("--cmake-args" "${python_cmake_arg}")
fi

if [[ ! -x "${ros_python_executable}" ]] ||
   ! "${ros_python_executable}" -c "import numpy" >/dev/null 2>&1; then
  print -u2 "Python cannot import NumPy: ${ros_python_executable}"
  print -u2 "Activate/source the Python environment used by ROS 2, then retry."
  exit 1
fi

colcon build --symlink-install --packages-select my_delta_robot "${build_args[@]}"

if [[ "${run_tests}" == true ]]; then
  colcon test --packages-select my_delta_robot
  colcon test-result --verbose
fi

print ""
print "Build OK. Source the workspace:"
print "  source install/setup.zsh"
