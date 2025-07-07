#!/bin/bash

trap_command_active=true

cleanup() {
    if [ "$trap_command_active" = true ]; then
        echo ""
        echo "SIGINT/SIGTERM caught, shutting down processes..."
        if [ -n "$CAMERA_PID" ]; then
            echo "Stopping RealSense camera node (PID: $CAMERA_PID)..."
            kill -SIGTERM "$CAMERA_PID" 2>/dev/null
        fi
        if [ -n "$PLANB_PID" ]; then
            echo "Stopping planb node (PID: $PLANB_PID)..."
            kill -SIGTERM "$PLANB_PID" 2>/dev/null
        fi
        if [ -n "$WATER_DROP_PID" ]; then
            echo "Stopping water_drop_mission node (PID: $WATER_DROP_PID)..."
            kill -SIGTERM "$WATER_DROP_PID" 2>/dev/null
        fi
        if [ -n "$BARREL_MANAGER_PID" ]; then
            echo "Stopping barrel_manager node (PID: $BARREL_MANAGER_PID)..."
            kill -SIGTERM "$BARREL_MANAGER_PID" 2>/dev/null
        fi
        wait
        echo "All processes stopped."
    fi
}

trap cleanup SIGINT SIGTERM

echo "🔍 启动 RealSense 相机节点..."
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true &
CAMERA_PID=$!

sleep 2

echo "Source 工作空间并启动 B/C/D 节点..."
cd ~/cqufly/fly2_ws
source install/setup.bash

echo "启动视觉检测节点 (planb)..."
ros2 run detect planb &
PLANB_PID=$!

echo "启动投水任务节点 (water_drop_mission)..."
ros2 run control water_drop_mission &
WATER_DROP_PID=$!

echo "启动桶管理器节点 (barrel_manager)..."
ros2 run barrel_manager barrel_manager &
BARREL_MANAGER_PID=$!

echo "---"
echo "All processes are running. Press Ctrl+C to stop all."

wait "$CAMERA_PID"
wait "$PLANB_PID"
wait "$WATER_DROP_PID"
wait "$BARREL_MANAGER_PID"

trap_command_active=false
cleanup
echo "Script finished."