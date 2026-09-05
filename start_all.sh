#!/usr/bin/env bash
# Start everything: hik camera + auto_aim container.
# Usage: bash /home/nvidia/game_v2/start_all.sh
set -e
for p in $(pgrep -f "component_container --ros-args"); do kill "$p" 2>/dev/null || true; done
for p in $(pgrep -f "all_nodes.launch.py"); do kill "$p" 2>/dev/null || true; done
for p in $(pgrep -f "hik_camera_node"); do kill "$p" 2>/dev/null || true; done
sleep 3
source /opt/ros/humble/setup.bash
cd /home/nvidia/game_v2 && source install/setup.bash
nohup ros2 run hik_camera hik_camera_node --ros-args --params-file /home/nvidia/game_v2/src/ros2-hik-camera/config/camera_params.yaml > /tmp/pipeline_hik.log 2>&1 &
sleep 3
nohup ros2 launch serical_device_ros2 all_nodes.launch.py > /tmp/pipeline_aim.log 2>&1 &
sleep 9
ros2 param set /auto_aim aim_mode direct >/dev/null 2>&1
echo "--- nodes ---"
ros2 node list 2>/dev/null | grep -E "auto_aim|hik_camera|vision_pub|robot_ctrl"
echo "--- vision ---"
timeout 3 ros2 topic echo /Vision_data --once 2>/dev/null | grep -m1 "^mode:"