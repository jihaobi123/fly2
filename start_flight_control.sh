#!/bin/bash

trap_command_active=true

cleanup() {
    if [ "$trap_command_active" = true ]; then
        echo ""
        echo "SIGINT/SIGTERM caught, shutting down processes..."
        if [ -n "$AGENT_PID" ]; then
            echo "Stopping MicroXRCEAgent (PID: $AGENT_PID)..."
            kill -SIGTERM "$AGENT_PID" 2>/dev/null
        fi
        if [ -n "$SWITCHD_PID" ]; then
            echo "Stopping switchd node (PID: $SWITCHD_PID)..."
            kill -SIGTERM "$SWITCHD_PID" 2>/dev/null
        fi
        wait
        echo "All processes stopped."
    fi
}

trap cleanup SIGINT SIGTERM

echo "🚁 启动 MicroXRCEAgent..."
MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600 &
AGENT_PID=$!
echo "MicroXRCEAgent started with PID: $AGENT_PID"

sleep 1

echo "🛩️ 启动航线控制节点 (switchd)..."
cd ~/cqufly/fly2_ws
source install/setup.bash
ros2 run detect switchd &
SWITCHD_PID=$!
echo "switchd node started with PID: $SWITCHD_PID"

echo "---"
echo "All processes are running. Press Ctrl+C to stop all."

wait "$AGENT_PID"
wait "$SWITCHD_PID"

trap_command_active=false
cleanup
echo "Script finished."