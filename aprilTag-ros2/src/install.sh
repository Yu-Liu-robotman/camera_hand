#!/usr/bin/env bash
# =============================================================================
# install.sh — camera_hand 手眼标定环境一键安装/编译
# 详见同目录 ../ENVIRONMENT.md
#
# 核心原则:
#   * colcon 编译用「系统 Python」(/usr/bin/python3)
#   * 跑标定节点用 conda 环境 (默认 handeye)
#   * 两者 Python 次版本必须一致 (Ubuntu 22.04 + ROS 2 Iron -> 3.10)
#
# 推荐用法:
#   cd aprilTag-ros2/src
#   source /opt/ros/iron/setup.bash
#   bash install.sh --rosdeps --udev --extras
#
# 选项:
#   --rosdeps          apt 安装构建/运行依赖 (需要 sudo)
#   --udev             安装 RealSense udev 规则 (需要 sudo)
#   --extras           补装 install.sh 默认不管的包:
#                      cyclonedds pygame pin(pinocchio) tlibot_sdkpy(editable)
#   --skip-build       只装依赖, 不做 colcon build
#   --ros-distro NAME  指定 ROS 2 版本 (默认自动探测, 如 iron/humble)
#   --conda-env NAME   指定 conda 环境名 (默认 handeye)
#   -h | --help        显示本帮助
# =============================================================================
set -eo pipefail

# ---------- 路径与常量 ----------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"   # .../aprilTag-ros2/src
WS_ROOT="$(dirname "$SCRIPT_DIR")"                            # .../aprilTag-ros2
CONDA_ENV_NAME="${CONDA_ENV_NAME:-handeye}"
CONDA_ENV_FILE="$SCRIPT_DIR/environment.yml"
REQUIREMENTS_FILE="$SCRIPT_DIR/requirements.txt"
UDEV_RULE_SRC="$SCRIPT_DIR/99-realsense.rules"
UDEV_RULE_DST="/etc/udev/rules.d/99-realsense.rules"
SETUP_PY="$SCRIPT_DIR/setup.py"
SETUP_PY_SRC="$SCRIPT_DIR/apriltags_ros_setup.py"
SYSTEM_PYTHON="/usr/bin/python3"
PKGS=(apriltags_ros aruco aruco_msgs aruco_ros)

# ---------- 开关 ----------
DO_ROSDEPS=0
DO_UDEV=0
DO_EXTRAS=0
DO_BUILD=1
ROS_DISTRO_ARG=""

# ---------- 输出helpers ----------
c_info() { printf '\033[1;34m[install]\033[0m %s\n' "$*"; }
c_warn() { printf '\033[1;33m[install]\033[0m %s\n' "$*"; }
c_die()  { printf '\033[1;31m[install]\033[0m %s\n' "$*" >&2; exit 1; }

usage() { sed -n '2,29p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0; }

# ---------- 解析参数 ----------
while [[ $# -gt 0 ]]; do
  case "$1" in
    --rosdeps)          DO_ROSDEPS=1 ;;
    --udev)             DO_UDEV=1 ;;
    --extras)           DO_EXTRAS=1 ;;
    --skip-build)       DO_BUILD=0 ;;
    --ros-distro)       ROS_DISTRO_ARG="${2:-}"; shift ;;
    --conda-env)        CONDA_ENV_NAME="${2:-}"; shift ;;
    -h|--help)          usage ;;
    *) c_die "未知参数: $1 (用 --help 查看用法)" ;;
  esac
  shift
done

# ---------- 探测 ROS 2 ----------
detect_ros_distro() {
  if [[ -n "$ROS_DISTRO_ARG" ]]; then echo "$ROS_DISTRO_ARG"; return; fi
  if [[ -n "${ROS_DISTRO:-}" ]]; then echo "$ROS_DISTRO"; return; fi
  local d
  for d in iron humble jazzy rolling galactic foxy; do
    if [[ -f "/opt/ros/$d/setup.bash" ]]; then echo "$d"; return; fi
  done
}
ROS_DISTRO="$(detect_ros_distro)"
[[ -n "$ROS_DISTRO" ]] || c_die "未探测到 ROS 2 安装 (未找到 /opt/ros/*/setup.bash)。请先安装 ROS 2 或用 --ros-distro 指定。"
ROS_SETUP="/opt/ros/$ROS_DISTRO/setup.bash"
c_info "ROS 2 版本: $ROS_DISTRO  ($ROS_SETUP)"

# ---------- ① apt 依赖 ----------
APT_PKGS=(
  "ros-${ROS_DISTRO}-desktop"
  "ros-${ROS_DISTRO}-realsense2-camera"
  "ros-${ROS_DISTRO}-realsense2-camera-msgs"
  "ros-${ROS_DISTRO}-librealsense2"
  "ros-${ROS_DISTRO}-cv-bridge"
  "ros-${ROS_DISTRO}-image-transport"
  "ros-${ROS_DISTRO}-tf2-ros"
  libopencv-dev
  libeigen3-dev
  python3-colcon-common-extensions
  python3-rosdep python3-catkin-pkg python3-empy python3-lark python3-yaml python3-numpy
  build-essential cmake git
)
install_rosdeps() {
  c_info "安装 apt 依赖 (需要 sudo)…"
  sudo apt-get update
  sudo apt-get install -y "${APT_PKGS[@]}"
  c_info "apt 依赖安装完成。"
}

# ---------- ② RealSense udev ----------
install_udev() {
  [[ -f "$UDEV_RULE_SRC" ]] || c_die "找不到 udev 规则文件: $UDEV_RULE_SRC"
  c_info "安装 RealSense udev 规则 (需要 sudo)…"
  sudo install -m 0644 "$UDEV_RULE_SRC" "$UDEV_RULE_DST"
  sudo udevadm control --reload-rules
  sudo udevadm trigger
  c_info "udev 规则已安装: $UDEV_RULE_DST (重新插拔相机生效)"
}

# ---------- ③ conda 环境 ----------
setup_conda_env() {
  command -v conda >/dev/null 2>&1 || c_die "未找到 conda。请先安装 miniconda/miniforge。"
  local conda_base
  conda_base="$(conda info --base)"
  # shellcheck disable=SC1090
  source "$conda_base/etc/profile.d/conda.sh"

  if conda env list | awk 'NF && $1 !~ /^#/ {print $1}' | grep -qx "$CONDA_ENV_NAME"; then
    c_info "conda 环境 '$CONDA_ENV_NAME' 已存在, 更新中…"
    conda env update -n "$CONDA_ENV_NAME" -f "$CONDA_ENV_FILE" --prune
  else
    c_info "创建 conda 环境 '$CONDA_ENV_NAME' (来自 $CONDA_ENV_FILE)…"
    conda env create -f "$CONDA_ENV_FILE"
  fi

  if [[ -f "$REQUIREMENTS_FILE" ]]; then
    c_info "安装 Python 依赖: $REQUIREMENTS_FILE"
    conda run -n "$CONDA_ENV_NAME" python -m pip install -r "$REQUIREMENTS_FILE"
  fi

  # apriltags_ros 可编辑安装 (仓库里 setup.py 存为 apriltags_ros_setup.py)
  if [[ ! -f "$SETUP_PY" && -f "$SETUP_PY_SRC" ]]; then
    cp "$SETUP_PY_SRC" "$SETUP_PY"
    c_info "由 apriltags_ros_setup.py 生成 setup.py"
  fi
  c_info "pip install -e . (apriltags_ros)"
  conda run -n "$CONDA_ENV_NAME" python -m pip install -e "$SCRIPT_DIR"
}

# ---------- ④ 补装包 (--extras) ----------
install_extras() {
  c_info "补装 SDK 运行依赖 (cyclonedds / pygame / pin)…"
  conda run -n "$CONDA_ENV_NAME" python -m pip install cyclonedds pygame pin
  c_info "pip install -e tlibot_sdk_python"
  conda run -n "$CONDA_ENV_NAME" python -m pip install -e "$SCRIPT_DIR/tlibot_sdk_python"
}

# ---------- ⑤ 脚本可执行位 ----------
chmod_scripts() {
  shopt -s nullglob
  local f
  for f in "$SCRIPT_DIR"/apriltags_ros/scripts/*.py; do
    chmod +x "$f" || true
  done
  shopt -u nullglob
}

# ---------- ⑥ colcon 编译 (系统 Python) ----------
build_ws() {
  command -v colcon >/dev/null 2>&1 || c_die "未找到 colcon。用 --rosdeps 安装 python3-colcon-common-extensions。"
  [[ -f "$ROS_SETUP" ]] || c_die "找不到 $ROS_SETUP"
  c_info "source $ROS_SETUP"
  set +u
  # shellcheck disable=SC1090
  source "$ROS_SETUP"
  set -eo pipefail

  c_info "colcon build (系统 Python: $SYSTEM_PYTHON), 包: ${PKGS[*]}"
  ( cd "$WS_ROOT" && colcon build \
      --packages-select "${PKGS[@]}" \
      --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DPython3_EXECUTABLE="$SYSTEM_PYTHON" \
        -DPYTHON_EXECUTABLE="$SYSTEM_PYTHON" )
  c_info "编译完成。产物在 $WS_ROOT/install"
}

# ============================= 主流程 =============================
c_info "脚本目录: $SCRIPT_DIR"
c_info "工作区根: $WS_ROOT"

if [[ $DO_ROSDEPS -eq 1 ]]; then install_rosdeps; fi
if [[ $DO_UDEV    -eq 1 ]]; then install_udev;   fi

setup_conda_env
if [[ $DO_EXTRAS  -eq 1 ]]; then install_extras; fi
chmod_scripts
if [[ $DO_BUILD   -eq 1 ]]; then build_ws;       fi

echo
c_info "全部完成！"
echo
echo "运行方式:"
echo "  conda activate $CONDA_ENV_NAME"
echo "  source /opt/ros/$ROS_DISTRO/setup.bash"
echo "  source $WS_ROOT/install/setup.bash"
echo "  ros2 launch apriltags_ros CameraTagRun.launch.py"
echo "  # 另开终端:"
echo "  python3 $SCRIPT_DIR/robot_model/base2end_pose.py"
if [[ $DO_EXTRAS -eq 0 ]]; then
  echo
  c_warn "未使用 --extras: 若运行 read_arm_joints.py / base2end_pose.py 报 ImportError,"
  c_warn "请补装 (关键，漏了必报错):"
  echo "  conda run -n $CONDA_ENV_NAME python -m pip install cyclonedds pygame pin"
  echo "  conda run -n $CONDA_ENV_NAME python -m pip install -e $SCRIPT_DIR/tlibot_sdk_python"
fi
