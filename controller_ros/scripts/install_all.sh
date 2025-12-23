#!/bin/bash
# ============================================================================
# TurtleBot1 控制器完整安装脚本
# 适用于: Ubuntu 20.04 + ROS Noetic
# 
# 功能: 一键安装 universal_controller + controller_ros + ACADOS
# 
# 使用方法:
#   chmod +x install_all.sh
#   ./install_all.sh
#
# 作者: Auto-generated
# 日期: 2024-12-23
# ============================================================================

set -e  # 遇到错误立即退出

# ============================================================================
# 颜色定义
# ============================================================================
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# ============================================================================
# 配置变量 (根据你的实际路径修改)
# ============================================================================
# AllControlBase 的路径 (包含 universal_controller 和 controller_ros)
ALLCONTROLBASE_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

# catkin 工作空间路径
CATKIN_WS="$HOME/catkin_ws"

# ACADOS 安装路径
ACADOS_INSTALL_DIR="$HOME/acados"

# ============================================================================
# 辅助函数
# ============================================================================
print_header() {
    echo ""
    echo -e "${BLUE}============================================================================${NC}"
    echo -e "${BLUE}  $1${NC}"
    echo -e "${BLUE}============================================================================${NC}"
    echo ""
}

print_success() {
    echo -e "${GREEN}✅ $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}⚠️  $1${NC}"
}

print_error() {
    echo -e "${RED}❌ $1${NC}"
}

print_info() {
    echo -e "${BLUE}ℹ️  $1${NC}"
}

check_command() {
    if command -v $1 &> /dev/null; then
        return 0
    else
        return 1
    fi
}

# ============================================================================
# 步骤 0: 检查环境
# ============================================================================
print_header "步骤 0: 检查环境"

# 检查 Ubuntu 版本
if [ -f /etc/os-release ]; then
    . /etc/os-release
    if [ "$VERSION_ID" != "20.04" ]; then
        print_warning "检测到 Ubuntu $VERSION_ID，本脚本针对 Ubuntu 20.04 优化"
    else
        print_success "Ubuntu 20.04 ✓"
    fi
fi

# 检查 ROS Noetic
if [ -f /opt/ros/noetic/setup.bash ]; then
    source /opt/ros/noetic/setup.bash
    print_success "ROS Noetic 已安装 ✓"
else
    print_error "ROS Noetic 未安装！请先安装 ROS Noetic"
    echo "安装命令: sudo apt install ros-noetic-desktop-full"
    exit 1
fi

# 检查 Python3
if check_command python3; then
    PYTHON_VERSION=$(python3 --version)
    print_success "Python3 已安装: $PYTHON_VERSION ✓"
else
    print_error "Python3 未安装！"
    exit 1
fi

# 检查 pip
if check_command pip3; then
    print_success "pip3 已安装 ✓"
else
    print_warning "pip3 未安装，正在安装..."
    sudo apt update
    sudo apt install -y python3-pip
fi

# 检查路径
if [ -d "$ALLCONTROLBASE_PATH/universal_controller" ] && [ -d "$ALLCONTROLBASE_PATH/controller_ros" ]; then
    print_success "AllControlBase 路径正确: $ALLCONTROLBASE_PATH ✓"
else
    print_error "找不到 universal_controller 或 controller_ros 目录"
    print_error "请确保脚本在 AllControlBase/controller_ros/scripts/ 目录下运行"
    print_error "或修改脚本中的 ALLCONTROLBASE_PATH 变量"
    exit 1
fi

# ============================================================================
# 步骤 1: 安装系统依赖
# ============================================================================
print_header "步骤 1: 安装系统依赖"

sudo apt update
sudo apt install -y \
    build-essential \
    cmake \
    git \
    python3-dev \
    python3-pip \
    python3-numpy \
    python3-scipy \
    python3-yaml \
    ros-noetic-tf2-ros \
    ros-noetic-nav-msgs \
    ros-noetic-sensor-msgs \
    ros-noetic-geometry-msgs \
    ros-noetic-std-srvs

print_success "系统依赖安装完成 ✓"

# ============================================================================
# 步骤 2: 升级 pip 并安装 Python 依赖
# ============================================================================
print_header "步骤 2: 升级 pip 并安装 Python 依赖"

# 升级 pip 到最新版本 (支持 pyproject.toml 可编辑安装需要 pip >= 21.3)
print_info "升级 pip 到最新版本..."
pip3 install --user --upgrade pip setuptools wheel

# 确保使用新版 pip
export PATH="$HOME/.local/bin:$PATH"

# 验证 pip 版本
PIP_VERSION=$(pip3 --version | awk '{print $2}')
print_success "pip 版本: $PIP_VERSION ✓"

# 安装 Python 依赖
pip3 install --user numpy scipy PyYAML matplotlib

print_success "Python 依赖安装完成 ✓"

# ============================================================================
# 步骤 3: 安装 ACADOS (高性能 MPC 求解器) - 必需
# ============================================================================
print_header "步骤 3: 安装 ACADOS (高性能 MPC 求解器)"

if [ -d "$ACADOS_INSTALL_DIR" ] && [ -f "$ACADOS_INSTALL_DIR/lib/libacados.so" ]; then
    print_warning "ACADOS 已存在于 $ACADOS_INSTALL_DIR，跳过安装"
else
    print_info "开始安装 ACADOS..."
    
    # 安装 ACADOS 编译依赖
    sudo apt install -y \
        liblapack-dev \
        libblas-dev \
        libboost-all-dev
    
    # 克隆 ACADOS
    cd $HOME
    if [ -d "$ACADOS_INSTALL_DIR" ]; then
        print_info "删除旧的 ACADOS 目录..."
        rm -rf "$ACADOS_INSTALL_DIR"
    fi
    
    print_info "克隆 ACADOS 仓库..."
    git clone https://github.com/acados/acados.git "$ACADOS_INSTALL_DIR"
    cd "$ACADOS_INSTALL_DIR"
    
    print_info "初始化子模块..."
    git submodule update --recursive --init
    
    # 编译 ACADOS
    print_info "编译 ACADOS (这可能需要几分钟)..."
    mkdir -p build
    cd build
    cmake -DACADOS_WITH_QPOASES=ON \
          -DACADOS_WITH_OSQP=ON \
          -DCMAKE_INSTALL_PREFIX="$ACADOS_INSTALL_DIR" \
          ..
    make -j$(nproc)
    make install
    
    print_success "ACADOS 编译完成 ✓"
fi

# 安装 ACADOS Python 接口
print_info "安装 ACADOS Python 接口..."
pip3 install --user "$ACADOS_INSTALL_DIR/interfaces/acados_template"

# 设置 ACADOS 环境变量
ACADOS_ENV_SETUP="
# ACADOS 环境变量
export ACADOS_SOURCE_DIR=$ACADOS_INSTALL_DIR
export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:$ACADOS_INSTALL_DIR/lib
"

# 检查是否已添加到 .bashrc
if ! grep -q "ACADOS_SOURCE_DIR" ~/.bashrc; then
    echo "$ACADOS_ENV_SETUP" >> ~/.bashrc
    print_success "ACADOS 环境变量已添加到 ~/.bashrc ✓"
else
    print_warning "ACADOS 环境变量已存在于 ~/.bashrc"
fi

# 立即生效
export ACADOS_SOURCE_DIR=$ACADOS_INSTALL_DIR
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$ACADOS_INSTALL_DIR/lib

print_success "ACADOS 安装完成 ✓"

# ============================================================================
# 步骤 4: 安装 universal_controller
# ============================================================================
print_header "步骤 4: 安装 universal_controller"

# pyproject.toml 在 AllControlBase 根目录
cd "$ALLCONTROLBASE_PATH"

# 可编辑安装 (从根目录安装，因为 pyproject.toml 在根目录)
print_info "从 $ALLCONTROLBASE_PATH 安装 universal_controller..."
pip3 install --user -e .

# 验证安装
print_info "验证 universal_controller 安装..."
python3 -c "
from universal_controller import ControllerManager, DEFAULT_CONFIG
from universal_controller.core.enums import ControllerState
print('universal_controller 导入成功')
" && print_success "universal_controller 安装验证通过 ✓" || {
    print_error "universal_controller 安装验证失败"
    exit 1
}

# ============================================================================
# 步骤 5: 创建 catkin 工作空间并编译 controller_ros
# ============================================================================
print_header "步骤 5: 编译 controller_ros"

# 创建 catkin 工作空间
if [ ! -d "$CATKIN_WS/src" ]; then
    print_info "创建 catkin 工作空间: $CATKIN_WS"
    mkdir -p "$CATKIN_WS/src"
    cd "$CATKIN_WS"
    source /opt/ros/noetic/setup.bash
    catkin_make
fi

# 链接 controller_ros
cd "$CATKIN_WS/src"
if [ -L "controller_ros" ]; then
    print_warning "controller_ros 链接已存在，重新创建..."
    rm controller_ros
fi
ln -s "$ALLCONTROLBASE_PATH/controller_ros" controller_ros
print_success "创建符号链接: controller_ros -> $ALLCONTROLBASE_PATH/controller_ros ✓"

# 编译
cd "$CATKIN_WS"
source /opt/ros/noetic/setup.bash
print_info "编译 catkin 工作空间..."
catkin_make

print_success "controller_ros 编译完成 ✓"

# ============================================================================
# 步骤 6: 配置环境变量
# ============================================================================
print_header "步骤 6: 配置环境变量"

# ROS 工作空间环境
ROS_ENV_SETUP="
# ROS Noetic + controller_ros 环境
source /opt/ros/noetic/setup.bash
source $CATKIN_WS/devel/setup.bash
"

if ! grep -q "$CATKIN_WS/devel/setup.bash" ~/.bashrc; then
    echo "$ROS_ENV_SETUP" >> ~/.bashrc
    print_success "ROS 环境变量已添加到 ~/.bashrc ✓"
else
    print_warning "ROS 环境变量已存在于 ~/.bashrc"
fi

# 立即生效
source /opt/ros/noetic/setup.bash
source "$CATKIN_WS/devel/setup.bash"

print_success "环境变量配置完成 ✓"

# ============================================================================
# 步骤 7: 验证安装
# ============================================================================
print_header "步骤 7: 验证安装"

# 验证 ROS 包
print_info "验证 controller_ros 包..."
if rospack find controller_ros &> /dev/null; then
    print_success "controller_ros 包可用 ✓"
else
    print_error "controller_ros 包不可用"
    exit 1
fi

# 验证消息
print_info "验证 ROS 消息..."
if rosmsg show controller_ros/LocalTrajectoryV4 &> /dev/null; then
    print_success "LocalTrajectoryV4 消息可用 ✓"
else
    print_error "LocalTrajectoryV4 消息不可用"
    exit 1
fi

if rosmsg show controller_ros/UnifiedCmd &> /dev/null; then
    print_success "UnifiedCmd 消息可用 ✓"
else
    print_error "UnifiedCmd 消息不可用"
    exit 1
fi

# 验证 ACADOS
print_info "验证 ACADOS..."
python3 -c "
try:
    from acados_template import AcadosOcp, AcadosOcpSolver
    print('ACADOS Python 接口可用')
except ImportError as e:
    print(f'ACADOS 导入失败: {e}')
    exit(1)
" && print_success "ACADOS 验证通过 ✓" || print_warning "ACADOS Python 接口可能需要重新登录后生效"

# 完整验证
print_info "完整功能验证..."
python3 -c "
from universal_controller import ControllerManager, DEFAULT_CONFIG
from universal_controller.core.enums import ControllerState
from universal_controller.core.ros_compat import ROS_AVAILABLE, TF2_AVAILABLE

config = DEFAULT_CONFIG.copy()
config['system']['platform'] = 'differential'
config['system']['ctrl_freq'] = 20

manager = ControllerManager(config)
manager.initialize_default_components()

print(f'平台: {config[\"system\"][\"platform\"]}')
print(f'控制频率: {config[\"system\"][\"ctrl_freq\"]} Hz')
print(f'ROS 可用: {ROS_AVAILABLE}')
print(f'TF2 可用: {TF2_AVAILABLE}')

manager.shutdown()
print('ControllerManager 初始化和关闭成功')
" && print_success "完整功能验证通过 ✓" || {
    print_error "完整功能验证失败"
    exit 1
}

# ============================================================================
# 安装完成
# ============================================================================
print_header "🎉 安装完成!"

echo ""
echo -e "${GREEN}所有组件已成功安装:${NC}"
echo "  ✅ ACADOS 高性能 MPC 求解器"
echo "  ✅ universal_controller 算法库"
echo "  ✅ controller_ros ROS 胶水层"
echo ""
echo -e "${YELLOW}重要: 请执行以下命令使环境变量生效:${NC}"
echo ""
echo "    source ~/.bashrc"
echo ""
echo -e "${BLUE}启动控制器:${NC}"
echo ""
echo "    # 终端 1: 启动 TurtleBot 驱动"
echo "    roslaunch turtlebot_bringup minimal.launch"
echo ""
echo "    # 终端 2: 启动控制器"
echo "    roslaunch controller_ros turtlebot1.launch"
echo ""
echo "    # 终端 3: 启动你的轨迹发布器"
echo "    rosrun your_package trajectory_publisher.py"
echo ""
echo -e "${BLUE}带 Dashboard 监控启动:${NC}"
echo ""
echo "    roslaunch controller_ros turtlebot1.launch dashboard:=true"
echo ""
echo -e "${GREEN}============================================================================${NC}"
echo -e "${GREEN}  安装路径汇总${NC}"
echo -e "${GREEN}============================================================================${NC}"
echo "  ACADOS:              $ACADOS_INSTALL_DIR"
echo "  universal_controller: $ALLCONTROLBASE_PATH/universal_controller"
echo "  controller_ros:       $ALLCONTROLBASE_PATH/controller_ros"
echo "  catkin 工作空间:      $CATKIN_WS"
echo ""
