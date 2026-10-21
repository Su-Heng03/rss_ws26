# 传感器数据到关节角度映射算法 - 最简单Demo

## 📋 项目概述

这是一个最简单的传感器数据到关节角度映射算法的Demo实现。该Demo不包含数据滤波、关节限位等复杂功能，专注于实现基本的映射功能，确保代码能够正常运行。

## 🎯 设计目标

- ✅ **简单性**：代码结构清晰，易于理解
- ✅ **可运行性**：确保代码能够正常运行
- ✅ **可扩展性**：为后续功能扩展预留接口

## 📁 文件结构

```
simple_demo/
├── simple_demo/                    # Python包目录
│   ├── __init__.py
│   ├── simple_mapping.py          # 映射算法模块
│   └── simple_mapper_node.py       # 主节点
├── launch/                         # Launch文件目录
│   └── simple_mapper.launch.py    # Launch文件
├── resource/                       # ROS 2资源目录
│   └── simple_demo
├── package.xml                     # ROS 2包描述文件
├── setup.py                        # Python包配置
├── setup.cfg                       # 配置文件
├── 系统架构设计.md                  # 系统架构详细设计文档
├── 算法流程设计.md                  # 算法流程详细设计文档
└── README.md                       # 本文件
```

## 🔄 系统架构

```
sim_publisher (esp_wifi_pub)
    ↓ 发布传感器数据 (CSV格式，11维)
    ↓ 话题: /esp/wifi/data
simple_mapper_node (本Demo)
    ↓ 订阅传感器数据
    ↓ 执行映射算法
    ↓ 发布关节角度
    ↓ 话题: /joint_trajectory_controller/command
机械臂控制器
    ↓ 接收关节角度命令
    ↓ 控制机械臂运动
```

## 📊 映射算法（方案B - 推荐）

### 输入数据
- **话题**: `/esp/wifi/data`
- **格式**: CSV字符串，11个浮点数
- **关键数据**:
  - `data[1]`: 旋转编码器角度（度）
  - `data[2]`: Roll角度（度）
  - `data[3]`: Pitch角度（度）
  - `data[4]`: Yaw角度（度）

### 映射公式
```python
关节1 = 旋转编码器角度 × 0.1 × (π/180)  # 度转弧度
关节2 = Roll角度 × 0.5 × (π/180)
关节3 = Pitch角度 × 0.5 × (π/180)
关节4 = Yaw角度 × 0.5 × (π/180)
关节5 = 0.0
关节6 = 0.0
```

### 输出数据
- **话题**: `/joint_trajectory_controller/command`
- **格式**: `trajectory_msgs/JointTrajectory`
- **关节名称**: `['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']`
- **单位**: 弧度 (rad)

## 📖 设计文档

详细设计请参考：
- **[系统架构设计.md](系统架构设计.md)** - 包含系统概述、数据格式、模块结构、算法设计等
- **[算法流程设计.md](算法流程设计.md)** - 包含详细的流程图、数据流图、时序图等

## 🚀 运行流程

### 方式1：使用ros2 run（推荐用于测试）

```bash
# 终端1：启动机械臂仿真
ros2 launch kortex_bringup gen3.launch.py use_fake_hardware:=true

# 终端2：启动传感器数据发布器
ros2 run esp_wifi_pub sim_publisher

# 终端3：启动映射节点
ros2 run simple_demo simple_mapper_node

# 终端4：可视化（可选）
rviz2
```

### 方式2：使用launch文件（推荐用于正式运行）

```bash
# 终端1：启动机械臂仿真
ros2 launch kortex_bringup gen3.launch.py use_fake_hardware:=true

# 终端2：启动传感器数据发布器
ros2 run esp_wifi_pub sim_publisher

# 终端3：使用launch文件启动映射节点
ros2 launch simple_demo simple_mapper.launch.py

# 或者自定义参数
ros2 launch simple_demo simple_mapper.launch.py \
  scale_encoder:=0.1 \
  scale_rpy:=0.5
```

## 🔍 验证方法

- 使用 `ros2 topic echo /joint_trajectory_controller/command` 查看发布的关节角度
- 使用 `ros2 topic echo /esp/wifi/data` 查看传感器数据
- 在RViz中观察机械臂运动

## 📝 设计说明

### 为什么选择这个方案？

1. **简单直观**：映射关系清晰，易于理解和调试
2. **充分利用传感器**：使用旋转编码器和RPY角度数据
3. **避免过度复杂**：关节5和6设为0，减少复杂度

### 关键参数

- `scale_encoder = 0.1`: 编码器缩放因子
- `scale_rpy = 0.5`: RPY角度缩放因子

这些参数可以根据实际需求调整。

## ⚠️ 注意事项

1. 本Demo不包含数据滤波功能
2. 本Demo不包含关节限位检查
3. 本Demo不包含异常值检测
4. 所有功能都是为了保持简单性，确保代码能够运行

## 🔮 后续扩展方向

虽然当前版本保持简单，但为后续扩展预留了接口：
- 可以添加数据滤波模块
- 可以添加关节限位检查
- 可以添加更复杂的映射策略
- 可以添加参数配置文件支持

## 📦 编译和安装

```bash
# 进入工作空间
cd /colcon_ws/overlay_ws

# 编译包
colcon build --packages-select simple_demo

# Source工作空间
source install/setup.bash
```

## ⚙️ 参数配置

节点支持以下ROS 2参数（可通过命令行或launch文件设置）：

- `sensor_topic` (默认: `/esp/wifi/data`): 传感器数据话题名称
- `joint_topic` (默认: `/joint_trajectory_controller/command`): 关节控制话题名称
- `scale_encoder` (默认: `0.1`): 编码器缩放因子
- `scale_rpy` (默认: `0.5`): RPY角度缩放因子

### 使用示例

```bash
# 使用默认参数
ros2 run simple_demo simple_mapper_node

# 自定义参数
ros2 run simple_demo simple_mapper_node --ros-args \
  -p sensor_topic:=/esp/wifi/data \
  -p joint_topic:=/joint_trajectory_controller/command \
  -p scale_encoder:=0.1 \
  -p scale_rpy:=0.5
```

## 🐛 调试技巧

### 查看话题数据
```bash
# 查看传感器数据
ros2 topic echo /esp/wifi/data

# 查看发布的关节角度
ros2 topic echo /joint_trajectory_controller/joint_trajectory

ros2 topic echo /joint_trajectory_controller/command

# 查看话题频率
ros2 topic hz /esp/wifi/data
ros2 topic hz /joint_trajectory_controller/command
```

### 查看节点信息
```bash
ros2 node info /simple_mapper_node
ros2 param list /simple_mapper_node
ros2 param get /simple_mapper_node scale_encoder
```

---

**状态**: ✅ 代码实现完成，可以编译和运行

