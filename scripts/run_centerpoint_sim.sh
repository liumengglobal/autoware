#!/usr/bin/env bash
# Start a minimal simulation of autoware_lidar_centerpoint with a synthetic
# pointcloud (CUDA blackboard, composable container) and optional RViz.
#
# Usage:
#   bash run_centerpoint_sim.sh [--rviz] [--no-container-restart]
RVZ_ON=0
for arg in "$@"; do
  case "$arg" in
    --rviz) RVZ_ON=1 ;;
    --no-container-restart) NO_RESTART=1 ;;
  esac
done

WORK="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$WORK/.." && pwd)"

source /opt/ros/jazzy/setup.bash
source "$ROOT/install/setup.bash"

DATA_PATH="/home/aw/data"
MODEL_NAME=centerpoint_tiny
CONTAINER=pointcloud_container
LOG=/tmp/centerpoint_sim

cleanup() {
  echo "[cleanup] stopping all centerpoint-sim processes..."
  pkill -9 -f "autoware_lidar_centerpoint_node" 2>/dev/null
  pkill -9 -f "ros2 launch autoware_lidar_centerpoint" 2>/dev/null
  pkill -9 -f "synthetic_cuda_pointcloud_publisher" 2>/dev/null
  pkill -9 -f "component_container" 2>/dev/null
  pkill -9 -f "static_transform_publisher" 2>/dev/null
  pkill -9 -f "rviz2" 2>/dev/null
}
trap cleanup EXIT

if [ "${NO_RESTART:-0}" != "1" ]; then
  cleanup
  sleep 2
fi

mkdir -p "$LOG"

echo "[1/5] static TF (map->base_link, base_link->lidar_link)"
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 map base_link > "$LOG/tf1.log" 2>&1 &
ros2 run tf2_ros static_transform_publisher 0 0 2.0 0 0 0 1 base_link lidar_link > "$LOG/tf2.log" 2>&1 &

echo "[2/5] component container: $CONTAINER"
ros2 run rclcpp_components component_container --ros-args -r "__node:=$CONTAINER" > "$LOG/container.log" 2>&1 &
sleep 3

echo "[3/5] load synthetic CUDA pointcloud publisher"
ros2 component load "/$CONTAINER" synthetic_cuda_pointcloud_publisher \
  "synthetic_cuda_pointcloud_publisher::SyntheticCudaPointCloudPublisher" > "$LOG/load_pub.log" 2>&1

echo "[4/5] launch lidar_centerpoint (composable in container)"
cd "$ROOT"
ros2 launch autoware_lidar_centerpoint lidar_centerpoint.launch.xml \
  data_path:="$DATA_PATH" model_name:="$MODEL_NAME" \
  use_pointcloud_container:=true pointcloud_container_name:="$CONTAINER" > "$LOG/centerpoint.log" 2>&1 &

if [ "$RVZ_ON" = "1" ]; then
  echo "[5/5] RViz2"
  rviz2 -d "$WORK/centerpoint_sim.rviz" > "$LOG/rviz.log" 2>&1 &
else
  echo "[5/5] (skip RViz, pass --rviz to enable)"
fi

echo ""
echo "Simulation running. Press Ctrl+C to stop."
echo "  - /objects          : ros2 topic echo /objects"
echo "  - /diagnostics      : ros2 topic echo /diagnostics"
echo "  - logs              : $LOG/"
echo ""
echo "NOTE: detection runs only while a /objects subscriber exists. Use --rviz (rviz2"
echo "      subscribes to /objects) or subscribe manually, otherwise the node is idle."
wait
