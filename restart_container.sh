#!/usr/bin/env bash
# Restart only the auto_aim container (camera node keeps running).
# Usage: bash /home/nvidia/game_v2/restart_container.sh
set -e
for p in $(pgrep -f "component_container --ros-args"); do kill "$p" 2>/dev/null || true; done
for p in $(pgrep -f "all_nodes.launch.py"); do kill "$p" 2>/dev/null || true; done
sleep 3
source /opt/ros/humble/setup.bash
cd /home/nvidia/game_v2 && source install/setup.bash
nohup ros2 launch serical_device_ros2 all_nodes.launch.py > /tmp/pipeline_aim.log 2>&1 &
sleep 9
ros2 node list 2>/dev/null | grep auto_aim
echo "container restarted"