#!/usr/bin/env bash
for p in $(pgrep -f "hik_camera_node"); do kill "$p" 2>/dev/null || true; done
sleep 2
source /opt/ros/humble/setup.bash
cd /home/nvidia/game_v2 && source install/setup.bash
nohup ros2 run hik_camera hik_camera_node --ros-args --params-file /home/nvidia/game_v2/src/ros2-hik-camera/config/camera_params.yaml > /tmp/pipeline_hik.log 2>&1 &
sleep 5
ros2 node list 2>/dev/null | grep hik_camera
echo "hik restarted"
