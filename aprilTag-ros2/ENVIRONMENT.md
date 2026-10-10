# 运行环境（实测可跑通，换机照此装）

> 核心：**编译用系统 Python，运行节点用 conda 环境**，两者 Python 次版本必须一致（都是 3.10）。

## 1. 平台

| 项 | 版本 |
|---|---|
| OS | Ubuntu 22.04.5 LTS |
| ROS 2 | Iron（`/opt/ros/iron`） |
| 系统 Python（编译 / `ros2` CLI） | `/usr/bin/python3` = 3.10.12 |
| conda 环境 `handeye`（跑标定节点） | Python 3.10.21 |

## 2. 系统 apt 依赖

| 包 | 版本 |
|---|---|
| ros-iron-desktop（含 rclpy / rviz 等） | 0.10.0 |
| ros-iron-realsense2-camera | 4.55.1 |
| ros-iron-realsense2-camera-msgs | 4.55.1 |
| ros-iron-librealsense2 | 2.55.1 |
| ros-iron-cv-bridge | 3.5.0 |
| ros-iron-image-transport | 4.2.4 |
| ros-iron-tf2-ros | 0.31.8 |
| **libopencv-dev**（`aruco_ros` 检测用） | **4.5.4** |
| libeigen3-dev | — |
| python3-colcon-common-extensions | 0.3.0 |
| python3-rosdep / python3-catkin-pkg / python3-empy / python3-lark / python3-yaml / python3-numpy | — |
| build-essential / cmake / git | — |

## 3. conda 环境 `handeye` 里的 Python 包

| 包 | 版本 | 用途 |
|---|---|---|
| python | 3.10.21 | 必须与 ROS 的 3.10 一致 |
| numpy | 1.26.4（**必须 <2**） | |
| **opencv-python**（>=4.5.4） | **4.11.0.86** | Python 侧 `cv2.calibrateHandEye` |
| PyYAML | 6.0.3 | 读 `camare_params.yaml` |
| cyclonedds | 11.0.1 | `tlibot_sdkpy` 的 DDS 后端 |
| pygame | 2.6.1 | `tlibot_sdkpy` utils 里 import |
| pinocchio（`pin`） | 4.1.0 | `base2end_pose.py` 正解 |
| tlibot_sdkpy | 0.1.0 | editable：`src/tlibot_sdk_python` |
| apriltags_ros | 0.0.1 | editable：`src` |

> `pin` 会自带 `cmeel-* / coal / eigenpy / boost / urdfdom` 等依赖，pip 自动装，无需手动。
> `mujoco`、`onnx/onnxruntime`、`nvidia-*` 等项目代码**用不到**，无需安装。
> ROS 的 Python 包（`rclpy`、`cv_bridge`、`rcl-interfaces` 等）由 `source /opt/ros/iron/setup.bash` 提供，**不要**用 pip 装。

## 4. colcon 编译的包

`apriltags_ros`、`aruco`、`aruco_msgs`、`aruco_ros`（`aruco` 是源码包，apt 里没有）

---

## 5. 一键安装

```bash
# ① ROS 2 Iron（若未装）
sudo apt install ros-iron-desktop

# ② 一键装：apt 依赖 + 相机 udev + conda 环境 + 编译
cd aprilTag-ros2/src
conda activate handeye
source /opt/ros/iron/setup.bash
bash install.sh --rosdeps --udev

# ③ 补装 install.sh 不管的（关键，漏了必报错）
conda run -n handeye python -m pip install cyclonedds pygame pin
conda run -n handeye python -m pip install -e tlibot_sdk_python
```

`install.sh` 已包含：apt 构建依赖、RealSense udev、建/更新 conda 环境、`pip install -r requirements.txt`、`pip install -e .`（apriltags_ros）、脚本加可执行位、用系统 Python 编译 4 个包。

## 6. 运行

```bash
conda activate handeye
source /opt/ros/iron/setup.bash
source install/setup.bash          # 在 aprilTag-ros2 目录下
ros2 launch apriltags_ros CameraTagRun.launch.py
# 另开终端：
python3 src/robot_model/base2end_pose.py
```

---

## 7. 必须注意（否则报错）

- **两套 Python**：`colcon build` 用 `/usr/bin/python3`，运行节点用 conda `handeye`；两边都必须是 **3.10**。
- **numpy 不要 >=2**（`numpy<2`）。
- **`cyclonedds` / `pygame` / `pin` / `tlibot_sdkpy` 必须手动补**（§5 第 ③ 步），否则 `read_arm_joints.py`、`base2end_pose.py` 直接 `ImportError`。
- 编译只编 4 个包 `--packages-select apriltags_ros aruco aruco_msgs aruco_ros`；`robot_model/`、`tlibot_sdk_python/` 没有 `package.xml`，不是 colcon 包，直接 `python3 xxx.py` 跑。
- 相机需要 udev 规则（`install.sh --udev`），否则要 sudo 才能开相机。
