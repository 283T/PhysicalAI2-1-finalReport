#!/usr/bin/env bash
# Fetch the reviewed upstream revision and build only the robot description.
set -euo pipefail
cd "$(dirname "$0")/.."
melfa_source=.cache/melfa_ros2_driver
melfa_revision=10098bed6bcca4e4bef46e36389a839065f8cb22
if [ ! -d "$melfa_source" ]; then
  mkdir -p "$melfa_source"
  git init "$melfa_source"
  git -C "$melfa_source" remote add origin https://github.com/MitsubishiElectric/melfa_ros2_driver.git
  git -C "$melfa_source" fetch --depth 1 origin "$melfa_revision"
  git -C "$melfa_source" checkout --detach FETCH_HEAD
fi
if [ "$(git -C "$melfa_source" rev-parse HEAD)" != "$melfa_revision" ]; then
  echo "MELFA checkout differs from the reviewed revision; use a separate clean cache directory." >&2
  exit 1
fi
docker compose run --rm dev bash -c \
  'colcon --log-base outputs/melfa_logs build --base-paths .cache/melfa_ros2_driver/melfa_description --packages-select melfa_description --build-base outputs/melfa_build --install-base outputs/melfa_install --cmake-args -DBUILD_TESTING=OFF'
