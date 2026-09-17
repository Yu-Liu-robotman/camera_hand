## 一、源码拉取（Git 方式）
### 1. 克隆项目仓库
```bash
git clone git@github.com:Yu-Liu-robotman/camera_hand.git
cd aprilTag-ros2
```

### 2. 本地 Git 基础操作（可选）
```bash
# 1. 进入你的项目目录
cd ~/aprilTag-ros2

# 2. 初始化Git仓库（如果还没初始化）
git init

# 3. 添加远程仓库
git remote add github git@github.com:Yu-Liu-robotman/camera_hand.git

# 4. 添加所有文件到暂存区
git add .

# 5. 提交第一次commit
git commit -m "Initial commit"

# 6. 创建main分支并推送
git branch -M main
git push -u github main

# 推送（根据实际仓库权限补充远程仓库配置）
git push

# 创建分支
git branch 分支名

# 切换到该分支
git checkout 分支名

# 将分支放到主分支
git reset --hard f5a3b0db8a47247d7c68dbe947d5d11b72856b26
```
# 安装
```bash
1、
cd aprilTag-ros2/src
bash install.sh 
2、
conda activate handeye
source /opt/ros/iron/setup.bash
source install/setup.bash
```

# 手眼标定
```bash
1、z1 -> fk
python3 src/robot_model/base2end_pose.py

2、main run
ros2 launch apriltags_ros CameraTagRun.launch.py

```
**采集**：图像窗口里 → 摆个姿态、停稳 → 按 `G`（终端显示「已记录第 N 组」）→ 换个姿态重复。
**≥3 组，建议 10+**；姿态要绕不同轴、差异大，别只做平移。

**求解**：按 `Q` → 终端打印矩阵 → 节点退出。

