# 双目视觉 WiFi → MoveIt 2 → Kinova Gen3 仿真系统

## 📋 项目概述

当前主链路使用 **Windows 双目摄像机 → HTTP/WiFi → ROS 2 Humble → 相机坐标转换 → MoveIt 2 → Kinova Gen3 fake_hardware**，同时在 **RViz 显示可旋转的目标方块**。

本 README 重点是：每次重新启动系统时需要打开哪些终端、启动哪些节点、按什么顺序启动，以及怎样逐段排查。旧的键盘 `xyz_mapper` 仍然保留，但**不与新版 `pose_bridge` 同时运行**。

> **当前运行模式：** 11 维数据中的前 7 位是 `[x, y, z, qx, qy, qz, qw]`（相机坐标系），末尾 4 位补零。`pose_bridge` 将它转换为 `base_link` 下的 `PoseStamped`；`target_visualizer` 显示位姿；`target_controller` 将目标交给 MoveIt。

## 🔄 系统架构与 Topic

```text
Windows（物理双目摄像头）
  python main.py
  └─ stereo.py + depth_estimation.py
     YOLO 分割 / 立体深度 / 上表面 SVD 位姿
     └─ HTTP /data: {"data": [x,y,z,qx,qy,qz,qw,0,0,0,0]}
                  │
                  ▼
Docker：esp_wifi_pub / esp_wifi_publisher
                  │ /esp/wifi/data     std_msgs/msg/String（JSON 原文）
                  ▼
        gen3_target_control / pose_bridge
        相机 optical 坐标系 → base_link（XYZ + 四元数）
                  │ /gen3/target_pose geometry_msgs/msg/PoseStamped
             ┌────┴───────────────────┐
             ▼                        ▼
        target_visualizer        target_controller
             │                        │
     /gen3/target_marker              ├─ /compute_ik
     visualization_msgs/Marker        └─ /move_action（OMPL）
             │                        │
             ▼                        ▼
        RViz 目标方块          joint_trajectory_controller
                                      │
                              Kinova Gen3 fake_hardware
                                      │ /joint_states、TF
                                      ▼
                                     RViz
```

**核心区别：** `pose_bridge` 不做键盘方向积分；它使用摄像头已经计算出的绝对 XYZ 和四元数。`target_visualizer` 不发送运动命令，只负责绘制方块。

## 📁 主要文件

```text
overlay_ws/src/
├── esp_wifi_pub/esp_wifi_pub/
│   ├── esp_wifi_publisher.py
│   └── sim_publisher.py
├── gen3_target_control/
│   ├── gen3_target_control/
│   │   ├── pose_bridge.py           # 新模式：相机位姿转换
│   │   ├── target_visualizer.py     # RViz 方块
│   │   ├── target_controller.py     # IK + MoveIt 规划/执行
│   │   ├── xyz_mapper.py           # 旧模式：键盘方向积分（不要同时启动）
│   │   └── ik_test.py
│   ├── setup.py
│   └── package.xml
└── gen3_moveit_config/
    ├── config/
    └── launch/
```

Windows 端的视觉模块还包括 `stereo.py`、`depth_estimation.py` 和标定文件 `config/stereo_calibration.npz`；这些通常在独立的 Windows Python 项目中，而非上面的 ROS 2 包里。

## 📊 WiFi 数据协议

HTTP `/data` 返回：

```json
{"data": [0.10, 0.05, 0.40, 0.0, 0.0, 0.0, 1.0, 0, 0, 0, 0]}
```

| 数据索引         | 意义                      | 单位/格式    |
| ------------ | ----------------------- | -------- |
| `data[0:3]`  | 物体上表面中心 XYZ（相机坐标）       | 米        |
| `data[3:7]`  | 物体上表面朝向 `(qx,qy,qz,qw)` | 四元数 xyzw |
| `data[7:11]` | 补位                      | 全部为 0    |

`stereo.py` 对双目 `Q` 矩阵重投影所得的毫米点云取表面中心，再除以 1000 变成米；四元数来自拟合平面法向量与主方向，不等同于机械臂工具的最终抓取姿态。目标丢失时视觉端可能发送 11 个零；**`pose_bridge` 必须拒绝零长度四元数**，不能把这包数据当成有效目标。

## 📐 相机安装外参与重要修正

当前暂定相机固定在机器人前方、从上往下看，使用以下**仿真假设**：

- 相机光学原点在 `base_link` 下的位置：`t = (0.40, 0.00, 0.90)` m。
- 镜头光轴 `+Z_camera` 竖直朝下（对应 `-Z_base`）。
- 图像顶部朝向机器人底座，假定 `+X_base` 指向机器人前方；图像向下对应 `+X_base`。
- 光学坐标 `+X_camera` 向图像右、`+Y_camera` 向图像下。

**⚠️ 修正旧版旋转矩阵：** 先前给出的

```python
[[0, 1, 0], [-1, 0, 0], [0, 0, -1]]
```

行列式为 **-1**，是反射矩阵而不是合法三维旋转，不能用来生成正确四元数。按上述右手坐标约定，正确的相机到基座旋转为：

```python
self.R = np.array([
    [0.0, 1.0,  0.0],
    [1.0, 0.0,  0.0],
    [0.0, 0.0, -1.0],
])
```

其行列式为 **+1**，位置公式是：

```text
X_base = 0.40 + Y_camera
Y_base = X_camera
Z_base = 0.90 - Z_camera
```

**如果你还没有修复 `pose_bridge.py`，请先修复再启动视觉姿态控制。** 实际画面方向、`base_link` 的前方定义与右手坐标方向，应通过 RViz / TF 标定确认；这套外参目前是人为指定，不是真实相机与机器人外参标定结果。

## 🚀 完整启动流程（推荐顺序）

> 每个长时间运行的命令都放在**独立终端**。示例容器名 `test`、Docker 工作区 `/overlay_ws`。下方共需要 Windows 侧 1 个服务，以及 Docker 侧 6 个长期运行的进程。

### 0. WSL：启动 Docker、准备环境

```bash
docker start test
docker exec -it test bash
```

在**每一个**新开的 Docker 终端执行：

```bash
source /opt/ros/humble/setup.bash
source /colcon_ws/install/setup.bash  # 若镜像中存在此工作区
source /overlay_ws/install/setup.bash
```

如果 `source /colcon_ws/install/setup.bash` 不存在，可跳过该行。推荐先完成下文的编译，确保安装了最新代码。

### 1. Docker 终端 1：Kinova Gen3 fake_hardware

```bash
ros2 launch kortex_bringup gen3.launch.py \
  robot_ip:=192.168.1.10 \
  use_fake_hardware:=true \
  launch_rviz:=false
```

启动机器人模型、`ros2_control`、`joint_state_broadcaster` 和 `joint_trajectory_controller`。此时是 **fake_hardware 仿真**。

### 2. Docker 终端 2：MoveIt 规划服务器

```bash
ros2 launch gen3_moveit_config move_group.launch.py
```

提供 `/compute_ik`、`/move_action` 和轨迹执行能力。

### 3. Docker 终端 3：RViz

```bash
ros2 launch gen3_moveit_config moveit_rviz.launch.py
```

在 RViz 左侧 `Displays`：

- 保留 `MotionPlanning` 中当前机器人（Scene Robot）。
- 添加 `Marker` Display，Topic 设为 **`/gen3/target_marker`**。
- 想隐藏橙色目标机械臂，可以关闭 `Planning Request → Query Goal State`；想隐藏动画轨迹，可关闭 `Planned Path`。
- `Fixed Frame` 需要能通过 TF 转换到 `base_link`。配置完成后保存 RViz Config。

### 4. Windows 终端 4：启动双目摄像机与 HTTP 服务

在 Windows 项目根目录：

```powershell
python main.py
```

确保这次选择的是**双目摄像机 `stereo.py` 输入**，而不是旧的 `KeyboardInput`。具体选择方式取决于你 Windows `main.py` 的现有配置，本 README 不假设它的启动参数。

Windows 浏览器检查（仅代表本机地址）：

```text
http://127.0.0.1:8000/data
```

应看到 11 维数据：`[x,y,z,qx,qy,qz,qw,0,0,0,0]`。容器访问 Windows 时需要实际可达的主机 IP，不能照搬容器内的 `127.0.0.1`。如果 Windows 程序只监听 localhost，还需要调整其监听地址或网络转发。

### 5. Docker 终端 5：WiFi → ROS 2

先核对可执行名：

```bash
ros2 pkg executables esp_wifi_pub
```

若安装入口叫 `esp_wifi_publisher`：

```bash
ros2 run esp_wifi_pub esp_wifi_publisher --ros-args \
  -p host:=192.168.1.11\
  -p port:=80 \
  -p path:=/data \
  -p hz:=2.0
```

把 `192.168.X.X` 替换为容器可访问的 Windows 主机 IP。复用你之前已经联通的网络参数，不修改 WiFi 协议。此节点发布 `/esp/wifi/data`（`std_msgs/msg/String`）。

### 6. Docker 终端 6：pose_bridge（代替旧 xyz_mapper）

```bash
ros2 run gen3_target_control pose_bridge
```

订阅 `/esp/wifi/data`，按相机外参转换 XYZ 和四元数，发布 `/gen3/target_pose`（`geometry_msgs/msg/PoseStamped`，坐标系为 `base_link`）。

**不能同时运行**：`ros2 run gen3_target_control xyz_mapper`。否则两个发布者会争用 `/gen3/target_pose`。

### 7. Docker 终端 7：target_visualizer（RViz 方块）

```bash
ros2 run gen3_target_control target_visualizer
```

订阅 `/gen3/target_pose`，发布 `/gen3/target_marker`。RViz 中需另外勾选对应 `Marker` Display，并确认 Topic 正确。

当前显示代码应包含：

```python
marker.type = Marker.CUBE
marker.pose = msg.pose
# 设为长方体可更清楚地观察旋转
marker.scale.x = 0.06
marker.scale.y = 0.03
marker.scale.z = 0.02
```

此节点只显示物体位姿，不控制机械臂。**推荐先在这一步确认方块位置和姿态正确**，再启动控制器。

### 8. Docker 终端 8：target_controller（MoveIt）

在确认目标数据和坐标转换正确后，再运行：

```bash
ros2 run gen3_target_control target_controller
```

该节点订阅 `/gen3/target_pose`，调用 IK、OMPL 和 MoveGroup，并根据 `plan_only` 选项决定是否尝试执行。

**应核对当前版本实现了两项修改：**

1. `target_callback` 缓存完整的 `x,y,z,qx,qy,qz,qw`，`plan_and_execute` 使用输入四元数，而非 `self.fixed_q`。
2. `create_timer(1.0, self.dispatch_target, ...)`，即 **1 秒检查一次新目标**（不是保证一秒规划或执行完成）；`busy` 仍会避免并行发起 MoveIt 任务。

视觉模块发送的是**物体上表面姿态**，而 MoveIt IK 的目标是 `end_effector_link` 工具姿态：目前两者只是共用位姿接口，真正抓取前仍需增加物体到工具的位姿偏移，并对输入做稳定性、可达性和安全检查。

## ✅ 启动速查表

| 顺序  | 所在环境    | 节点 / 进程                    | 启动命令                                                   |
| --- | ------- | -------------------------- | ------------------------------------------------------ |
| 1   | Docker  | 机器人 fake_hardware          | `ros2 launch kortex_bringup gen3.launch.py ...`        |
| 2   | Docker  | MoveIt `move_group`        | `ros2 launch gen3_moveit_config move_group.launch.py`  |
| 3   | Docker  | RViz                       | `ros2 launch gen3_moveit_config moveit_rviz.launch.py` |
| 4   | Windows | 双目视觉 + HTTP                | `python main.py`（启用 stereo 模式）                         |
| 5   | Docker  | WiFi 发布器                   | `ros2 run esp_wifi_pub esp_wifi_publisher ...`         |
| 6   | Docker  | 坐标转换 `pose_bridge`         | `ros2 run gen3_target_control pose_bridge`             |
| 7   | Docker  | 方块可视化 `target_visualizer`  | `ros2 run gen3_target_control target_visualizer`       |
| 8   | Docker  | 运动目标控制 `target_controller` | `ros2 run gen3_target_control target_controller`       |

## 🔍 分段验证与故障排查

依次在新 Docker 终端中运行：

```bash
# A. WiFi 原始 JSON 是否到达 ROS 2
ros2 topic echo /esp/wifi/data

# B. 相机坐标转换后的 PoseStamped
ros2 topic echo /gen3/target_pose

# C. 检查发布/订阅关系
ros2 topic info /gen3/target_pose -v

# D. 可视化输出是否存在（需要有新的目标消息）
ros2 topic echo /gen3/target_marker --once

# E. 控制器是否 active
ros2 control list_controllers

# F. 规划与执行 Action 是否存在
ros2 action list -t

# G. 查看 fake_hardware 关节是否真的运动
ros2 topic echo /joint_states --field position
```

**正常 Topic 关系：** `/gen3/target_pose` 应有 **1 个发布者**（`pose_bridge`）和 **2 个订阅者**（`target_visualizer`、`target_controller`）；尚未启动控制器时只有 1 个订阅者。

**排查现象：**

| 现象                                           | 优先检查                                                                                  |
| -------------------------------------------- | ------------------------------------------------------------------------------------- |
| `/esp/wifi/data` 没消息                         | Windows `/data`、主机 IP、端口、ESP WiFi 发布器日志                                               |
| `/esp/wifi/data` 有数据但 `/gen3/target_pose` 没有 | `pose_bridge` 是否运行、四元数是否全零、JSON 是否为 11 维、日志报错                                         |
| 方块不显示                                        | `target_visualizer` 是否启动、RViz `Marker` Topic 是否为 `/gen3/target_marker`、Fixed Frame/TF |
| 方块位置反向或姿态不对                                  | 相机安装方向、`pose_bridge` 外参矩阵、光学坐标系与 base_link 坐标约定                                       |
| RViz 显示规划但关节不动                               | `plan_only`、MoveIt 执行 Action 结果、`joint_trajectory_controller`、`/joint_states`         |
| 输入抖动或突然跳跃                                    | 双目 YOLO Mask、深度质量、法向量符号与 SVD 主方向稳定性                                                   |

## 🧪 不接摄像机的独立测试

**测试时先关闭 `target_controller`**，避免测试用的位姿触发规划或运动。

一个终端先打开：

```bash
ros2 topic echo /gen3/target_pose
```

另一个终端发送相机坐标下的测试包：

```bash
ros2 topic pub --once /esp/wifi/data std_msgs/msg/String \
  '{"data": "{\"data\": [0.10, 0.05, 0.40, 0, 0, 0, 1, 0, 0, 0, 0]}"}'
```

按**本 README 中修正后的外参**，预期位置：`base_link` 下 **(0.45, 0.10, 0.50) m**；RViz 方块应出现在同一位置。旧版错误矩阵对应的负 Y 结果不可用作姿态变换。

## 📦 编译与加载

若 `pose_bridge.py`、`target_visualizer.py`、`target_controller.py`、`setup.py` 或 `package.xml` 发生变化：

```bash
cd /overlay_ws
colcon build --packages-select gen3_target_control
source install/setup.bash
```

若 WiFi 包也有改动：

```bash
colcon build --packages-select esp_wifi_pub
source install/setup.bash
```

首次部署时检查 `setup.py` 中注册了 `pose_bridge`、`target_visualizer`、`target_controller`。相应 `package.xml` 需要 `rclpy`、`std_msgs`、`geometry_msgs`、`visualization_msgs`、`moveit_msgs`；使用 NumPy 的 `pose_bridge` 还需要 Python NumPy 运行环境和相应依赖声明。修改后需**重启节点**，旧进程不会自动更新代码。

## ⚠️ 安全与实现边界

1. 当前仅限 `use_fake_hardware:=true` 进行验证；尚未完成真实机器人运动安全配置。
2. 四元数必须有效且归一化。视觉丢失时的 11 个零应被忽略，而不是当成零坐标执行。
3. **1 秒是目标检查周期**，不是 MoveIt 硬实时伺服周期；一次规划和执行可能明显长于 1 秒。
4. 之前已确认 Topic、IK 与 MoveIt 规划可以运行，但自动轨迹执行仍需要用 `/joint_states` 和 Action 结果独立确认。
5. 视频物体位姿不是工具抓取位姿；需进一步标定抓取偏移、设置可达空间和碰撞约束。
6. `pose_bridge` 目前采用仿真假定外参；调整相机安装位置时改平移，调整安装角度时还必须改旋转矩阵，并保持 `det(R)=+1`。
7. `target_visualizer` 的 Marker 是“当前目标”，并不是历史轨迹。要保留运动轨迹需另建轨迹 Marker。
8. **不要同时运行 `pose_bridge` 与 `xyz_mapper`**。旧键盘模式可单独切换回来，但它采用不同的 11 维数据解释方式。

---

**环境：** WSL2 / Docker `test` / ROS 2 Humble / MoveIt 2 / Kinova Gen3 / RViz
**模式：** 双目视觉 XYZ + Quaternion → Camera-to-Base → MoveIt + RViz Marker
**状态：** 新链路为当前目标架构；执行与相机外参应按上述步骤逐段验证。
