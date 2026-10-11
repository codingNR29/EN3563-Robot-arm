#!/bin/bash
# kill_sim.sh — Hard-kill all GP7 simulation processes
# Usage: bash ~/ros2_ws/src/kill_sim.sh

echo "🛑 Killing all Gazebo and ROS 2 simulation processes..."

pkill -9 -f "gz sim"
pkill -9 -f "gz_sim"
pkill -9 -f "gzserver"
pkill -9 -f "gzclient"
pkill -9 ruby
pkill -9 -f "ros2 launch"
pkill -9 -f "ros2 run"
pkill -9 -f "ros2 topic pub"
pkill -9 -f "robot_state_publisher"
pkill -9 -f "spawner"
pkill -9 -f "arm_controller_node"
pkill -9 -f "gripper_controller_node"
pkill -9 -f "demo_sequence"
pkill -9 -f "controller_manager"

sleep 1

# Verify
REMAINING=$(pgrep -a -f "gz|ros2 launch|arm_controller|gripper_controller" 2>/dev/null)
if [ -z "$REMAINING" ]; then
    echo "✅ All simulation processes killed. Safe to relaunch."
else
    echo "⚠️  Some processes still running:"
    echo "$REMAINING"
fi
