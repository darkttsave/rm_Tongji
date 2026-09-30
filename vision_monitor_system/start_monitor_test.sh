#!/bin/bash

# 快速启动测试脚本
# 同时启动监控程序和测试数据发送器

echo "════════════════════════════════════════════════════════════"
echo "  RoboMaster 视觉监控系统 - 测试模式"
echo "════════════════════════════════════════════════════════════"
echo ""
echo "正在启动监控程序..."
echo ""

# 切换到脚本所在目录
cd "$(dirname "$0")"

# 检查Python脚本是否存在
if [ ! -f "vision_monitor.py" ]; then
    echo "❌ 错误: 未找到 vision_monitor.py"
    echo "请确保在 vision_monitor_system 目录下运行此脚本"
    exit 1
fi

if [ ! -f "test_data_sender.py" ]; then
    echo "❌ 错误: 未找到 test_data_sender.py"
    exit 1
fi

# 在后台启动监控程序
echo "[1/2] 启动监控程序..."
python3 vision_monitor.py &
MONITOR_PID=$!

# 等待监控程序启动
sleep 2

# 启动测试数据发送器
echo "[2/2] 启动测试数据发送器..."
echo ""
echo "────────────────────────────────────────────────────────────"
echo "测试场景选择:"
echo "  1. 正常小陀螺 (normal)"
echo "  2. 静态目标 (static)"
echo "  3. 高速小陀螺 (fast_spin)"
echo "  4. 装甲板切换 (switch)"
echo "────────────────────────────────────────────────────────────"
echo ""

read -p "请选择场景 [1-4, 默认1]: " choice

case $choice in
    2)
        SCENARIO="static"
        ;;
    3)
        SCENARIO="fast_spin"
        ;;
    4)
        SCENARIO="switch"
        ;;
    *)
        SCENARIO="normal"
        ;;
esac

echo ""
echo "启动场景: $SCENARIO"
echo "按 Ctrl+C 停止测试"
echo ""

# 清理函数
cleanup() {
    echo ""
    echo "正在停止监控程序..."
    kill $MONITOR_PID 2>/dev/null
    echo "测试结束"
    exit 0
}

trap cleanup INT TERM

# 启动测试数据发送器
python3 test_data_sender.py --scenario $SCENARIO

# 如果数据发送器退出，也停止监控程序
cleanup
