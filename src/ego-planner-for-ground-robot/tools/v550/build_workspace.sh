#!/usr/bin/env bash
# Build only from this workspace and the system ROS installation.
set -eo pipefail
tool_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_dir="$(cd -- "$tool_dir/../.." && pwd)"
workspace_dir="$(cd -- "$project_dir/../.." && pwd)"
if [[ ! -d "$workspace_dir/src" ]]; then
  echo "Place the project under <workspace>/src before running this script." >&2
  exit 1
fi
case "${1:-real}" in
  real) target_package=ego_wheeltec ;;
  sim) target_package=ego_ground_sim ;;
  *) echo "Usage: $0 [real|sim]" >&2; exit 2 ;;
esac
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble is required: /opt/ros/humble/setup.bash is missing." >&2
  exit 1
fi
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash
export MAKEFLAGS="${MAKEFLAGS:--j${V550_BUILD_WORKERS:-2} -l${V550_BUILD_WORKERS:-2}}"
cd -- "$workspace_dir"
colcon build --packages-up-to "$target_package" --parallel-workers "${V550_BUILD_WORKERS:-2}" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
echo "Build complete. In each new terminal:"
echo "source /opt/ros/humble/setup.bash"
echo "source \"$workspace_dir/install/local_setup.bash\""
