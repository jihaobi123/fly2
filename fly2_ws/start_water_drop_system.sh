#!/bin/bash

# 投水系统启动脚本
# 使用方法: ./start_water_drop_system.sh

echo "🚁 启动投水系统..."

# 设置工作空间环境
source ~/cqufly/fly2_ws/install/setup.bash

# 检查是否已经source了环境
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ 错误: 请先source ROS2环境"
    echo "请运行: source /opt/ros/humble/setup.bash"
    exit 1
fi

echo "✅ ROS2环境已加载: $ROS_DISTRO"

# 启动PX4通信代理（根据你的配置调整）
echo "📡 启动PX4通信代理..."
# 如果使用microRTPS agent:
# micrortps_agent -d /dev/ttyACM0 -b 921600 &
# 如果使用MAVROS:
# ros2 launch mavros px4.launch.py fcu_url:=/dev/ttyACM0:921600 &

# 等待通信建立
sleep 2

echo "🔍 启动视觉检测节点 (planb)..."
ros2 run detect planb &
PLANB_PID=$!

echo "🛩️ 启动航线控制节点 (switchd)..."
ros2 run detect switchd &
SWITCHD_PID=$!

echo "🎯 启动投水任务节点 (water_drop_mission)..."
ros2 run control water_drop_mission &
WATER_DROP_PID=$!

echo "📊 启动桶管理器节点 (barrel_manager)..."
ros2 run barrel_manager barrel_manager &
BARREL_MANAGER_PID=$!

echo "✅ 所有节点已启动!"
echo "📋 节点PID:"
echo "  - planb: $PLANB_PID"
echo "  - switchd: $SWITCHD_PID"
echo "  - water_drop_mission: $WATER_DROP_PID"
echo "  - barrel_manager: $BARREL_MANAGER_PID"

echo ""
echo "🎮 系统已就绪，等待任务开始..."
echo "💡 提示: 按 Ctrl+C 停止所有节点"

# 等待所有后台进程
wait

echo "🛑 系统已停止" 