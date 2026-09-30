#!/bin/bash

# RoboMaster视觉监控系统安装脚本
# 适用于Ubuntu 22.04

set -e  # 遇到错误立即退出

echo "════════════════════════════════════════════════════════════"
echo "  RoboMaster 视觉监控系统安装程序"
echo "  同济大学SuperPower战队"
echo "════════════════════════════════════════════════════════════"
echo ""

# 检查Python版本
echo "[1/4] 检查Python环境..."
if ! command -v python3 &> /dev/null; then
    echo "❌ 未找到Python3，正在安装..."
    sudo apt update
    sudo apt install -y python3 python3-pip
else
    PYTHON_VERSION=$(python3 --version)
    echo "✓ 找到 $PYTHON_VERSION"
fi

# 检查pip
if ! command -v pip3 &> /dev/null; then
    echo "❌ 未找到pip3，正在安装..."
    sudo apt install -y python3-pip
else
    echo "✓ pip3 已安装"
fi

# 安装Python依赖
echo ""
echo "[2/4] 安装Python依赖包..."
echo "正在安装: numpy matplotlib..."

pip3 install --user numpy matplotlib || {
    echo "⚠️  使用pip安装失败，尝试使用apt..."
    sudo apt install -y python3-numpy python3-matplotlib
}

# 检查可选依赖
echo ""
echo "[3/4] 检查可选依赖..."
if pip3 list | grep -q PyQt5; then
    echo "✓ PyQt5 已安装（可选）"
else
    echo "ℹ PyQt5 未安装（可选，不影响使用）"
    echo "  如需安装: pip3 install --user PyQt5"
fi

# 设置权限
echo ""
echo "[4/4] 设置文件权限..."
chmod +x vision_monitor.py
echo "✓ 已设置执行权限"

# 测试安装
echo ""
echo "════════════════════════════════════════════════════════════"
echo "安装完成！正在测试..."
echo "════════════════════════════════════════════════════════════"
echo ""

python3 -c "import numpy; import matplotlib; print('✓ 所有依赖导入成功')" || {
    echo "❌ 依赖导入失败，请检查安装"
    exit 1
}

echo ""
echo "════════════════════════════════════════════════════════════"
echo "✓ 安装成功！"
echo "════════════════════════════════════════════════════════════"
echo ""
echo "使用方法："
echo "  1. 启动监控程序："
echo "     python3 vision_monitor.py"
echo ""
echo "  2. 在C++代码中集成（参考 monitor_integration_example.cpp）"
echo ""
echo "  3. 编译并运行你的C++程序"
echo ""
echo "详细说明请查看: MONITOR_README.md"
echo ""
echo "════════════════════════════════════════════════════════════"
