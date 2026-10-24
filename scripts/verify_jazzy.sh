#!/usr/bin/env bash
# Build/test in an isolated container; no robot, host network, or writable source mount.
set -euo pipefail
repo_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
verify_dir=${DWVP_VERIFY_WORKSPACE:-$(mktemp -d /tmp/ec-dwvp-jazzy-verify.XXXXXX)}
mkdir -p "$verify_dir/src"
image_name=${DWVP_JAZZY_IMAGE:-decwest/ytlab_castella_jazzy}
docker run --rm --network none --user "$(id -u):$(id -g)" \
  -e ROS_HOME=/tmp/ros -e ROS_LOCALHOST_ONLY=1 \
  -e ROS_DOMAIN_ID=179 -e PYTHONDONTWRITEBYTECODE=1 \
  -e AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1 \
  --entrypoint bash \
  -v "$verify_dir:/ws" \
  -v "$repo_dir:/ws/src/nav2_ec_dwvp_controller:ro" \
  -w /ws "$image_name" -c '
    source /opt/ros/jazzy/setup.bash
    colcon build --packages-select nav2_ec_dwvp_controller --cmake-args -DBUILD_TESTING=ON &&
    source install/setup.bash &&
    colcon test --packages-select nav2_ec_dwvp_controller --return-code-on-test-failure &&
    colcon test-result --verbose
  '
printf 'Build and test artifacts: %s\n' "$verify_dir"
