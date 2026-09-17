#!/bin/bash
# 手眼标定环境一键安装: bash install.sh [--rosdeps] [--udev] [--no-conda]
# 编译和 ros2 CLI 用系统 python(/usr/bin/python3)，标定节点用 conda 环境的 python。
# 运行: conda activate handeye && source /opt/ros/$ROS_DISTRO/setup.bash && source install/setup.bash
set -e

ENV_NAME=handeye
SRC_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SRC_DIR")"
ROS_DISTRO="${ROS_DISTRO:-iron}"
BUILD_PY=/usr/bin/python3

WITH_ROSDEPS=0
USE_CONDA=1
WITH_UDEV=0
for arg in "$@"; do
    case "$arg" in
        --rosdeps)  WITH_ROSDEPS=1 ;;
        --udev)     WITH_UDEV=1 ;;
        --no-conda) USE_CONDA=0 ;;
        *) echo "未知参数: $arg（--rosdeps / --udev / --no-conda）"; exit 1 ;;
    esac
done

if [ "$WITH_UDEV" = 1 ]; then
    sudo cp "$SRC_DIR/99-realsense.rules" /etc/udev/rules.d/99-realsense.rules
    sudo udevadm control --reload-rules
    sudo udevadm trigger --action=add --subsystem-match=usb
    echo "udev 规则已安装（拔插相机生效）"
fi

if [ "$USE_CONDA" = 1 ]; then
    command -v conda >/dev/null || { echo "✗ 未找到 conda"; exit 1; }
    if conda env list | awk '{print $1}' | grep -qx "$ENV_NAME"; then
        conda env update -n "$ENV_NAME" -f "$SRC_DIR/environment.yml"
    else
        conda env create -f "$SRC_DIR/environment.yml"
    fi
    PY_CMD="conda run -n $ENV_NAME --no-capture-output python"
    PIP_FLAGS=""
else
    PY_CMD="python3"
    PIP_FLAGS="--user"
fi
PIP_CMD="$PY_CMD -m pip"

# 节点的 python 次版本必须和 ROS 2 一致，否则 import rclpy 一定失败
ROS_PY="$(basename "$(ls -d /opt/ros/"$ROS_DISTRO"/lib/python3.* 2>/dev/null | head -1)" | sed 's/^python//')"
if [ -z "$ROS_PY" ]; then
    echo "✗ 找不到 /opt/ros/$ROS_DISTRO（可 ROS_DISTRO=xxx bash install.sh）"
    exit 1
fi
ENV_PY="$($PY_CMD -c 'import sys; print("%d.%d" % sys.version_info[:2])')"
if [ "$ROS_PY" != "$ENV_PY" ]; then
    echo "✗ python 版本不匹配: ROS 2=$ROS_PY，当前环境=$ENV_PY"
    exit 1
fi

if [ "$WITH_ROSDEPS" = 1 ]; then
    # shellcheck disable=SC1090
    source "/opt/ros/$ROS_DISTRO/setup.bash"
    sudo apt-get update
    sudo apt-get install -y python3-rosdep python3-colcon-common-extensions \
        build-essential cmake git python3-catkin-pkg python3-empy python3-lark \
        python3-yaml python3-numpy libopencv-dev libeigen3-dev
    [ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || sudo rosdep init
    rosdep update
    rosdep install --from-paths "$SRC_DIR" --ignore-src -y --rosdistro "$ROS_DISTRO"
fi

$PIP_CMD install $PIP_FLAGS -r "$SRC_DIR/requirements.txt"
cd "$SRC_DIR"
cp apriltags_ros_setup.py setup.py
$PIP_CMD install $PIP_FLAGS -e .
rm -f setup.py
chmod +x "$SRC_DIR"/apriltags_ros/scripts/*.py   # launch 用 which() 定位节点，必须有可执行位

$BUILD_PY -c "import catkin_pkg" 2>/dev/null || {
    echo "✗ 缺 catkin_pkg，请先: sudo apt install python3-catkin-pkg python3-lark"
    exit 1
}

# shellcheck disable=SC1090
source "/opt/ros/$ROS_DISTRO/setup.bash"
cd "$WS_DIR"
# realsense2_camera 用 apt 版（ros-$ROS_DISTRO-realsense2-camera），不从源码编
PKGS="apriltags_ros aruco aruco_msgs aruco_ros"
# /usr/bin 提前，防止 CMake 抓到 conda 的 python
PATH="/usr/bin:/bin:$PATH" colcon build --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE="$BUILD_PY" \
    --packages-select $PKGS

echo
$PY_CMD -c "import rclpy, cv_bridge, apriltags_ros; print('节点环境 OK')"
$BUILD_PY -c "import yaml, launch, launch_ros; print('ros2 CLI 环境 OK')"


conda activate handeye