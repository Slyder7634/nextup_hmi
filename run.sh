#!/bin/bash

source /opt/ros/humble/setup.bash
source ~/nextup_controllers/install/setup.bash
source ~/x_cat/install/setup.bash
source ~/NextupRobot/install/setup.bash

echo "Starting robot..."

ros2 launch simulation_nextup_moveit_config everything.launch.py rviz:=false \
    > /tmp/robot_launch.log 2>&1 &

LAUNCH_PID=$!

sleep 5

ros2 service call /servo_node/start_servo std_srvs/srv/Trigger "{}"

echo "Starting HMI..."

python3 main.py

# Optional: kill launch when HMI exits
kill $LAUNCH_PID