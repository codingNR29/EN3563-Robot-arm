#!/usr/bin/env bash
# Launch this checkout without Python/ament packages from stale ROS overlays.
set -e
GP7_SOURCE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ! -f "$GP7_SOURCE_ROOT/install/local_setup.bash" ]]; then
    echo "Build first: cd $GP7_SOURCE_ROOT && source /opt/ros/jazzy/setup.bash && colcon build --base-paths gp7_ros2 --symlink-install" >&2
    exit 1
fi
unset AMENT_PREFIX_PATH PYTHONPATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
source /opt/ros/jazzy/setup.bash
source "$GP7_SOURCE_ROOT/install/local_setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-87}"
export GZ_PARTITION="${GZ_PARTITION:-gp7_pick_place}"
exec ros2 launch gp7_ros2_pkg pick_place.launch.py "$@"
