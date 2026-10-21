# 传感器数据到关节角度映射算法 - 最简单Demo架构设计

## 1. 系统概述

本Demo实现最简单的传感器数据到关节角度的映射算法，用于验证基本功能。不包含数据滤波、关节限位等复杂功能。

### 1.1 设计目标
- **简单性**：代码结构清晰，易于理解
- **可运行性**：确保代码能够正常运行
- **可扩展性**：为后续功能扩展预留接口

### 1.2 系统组成
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

## 2. 数据格式

### 2.1 输入数据格式（来自sim_publisher）
- **话题名称**: `/esp/wifi/data`
- **消息类型**: `std_msgs/String`
- **数据格式**: CSV字符串，11个浮点数，逗号分隔
- **数据含义**:
  ```
  data[0]: 红外传感器 (1=无遮挡, 0=有遮挡)
  data[1]: 旋转编码器 (度，步长18°)
  data[2]: Roll角度 (度)
  data[3]: Pitch角度 (度)
  data[4]: Yaw角度 (度)
  data[5]: 加速度X轴重力余弦值 (-1到1)
  data[6]: 加速度Y轴重力余弦值 (-1到1)
  data[7]: 加速度Z轴重力余弦值 (-1到1)
  data[8]: 陀螺仪X轴角速度
  data[9]: 陀螺仪Y轴角速度
  data[10]: 陀螺仪Z轴角速度
  ```

### 2.2 输出数据格式（发送给机械臂）
- **话题名称**: `/joint_trajectory_controller/command`
- **消息类型**: `trajectory_msgs/JointTrajectory`
- **关节名称**: `['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']`
- **数据单位**: 弧度 (rad)

## 3. 系统架构

### 3.1 模块结构
```
simple_demo/
├── simple_mapper_node.py      # 主节点文件
├── simple_mapping.py          # 映射算法模块
├── setup.py                   # ROS 2包配置
├── package.xml                # 包描述文件
└── README.md                  # 使用说明
```

### 3.2 节点架构
```
SimpleMapperNode (ROS 2 Node)
├── 订阅者 (Subscriber)
│   └── 订阅 /esp/wifi/data 话题
├── 发布者 (Publisher)
│   └── 发布到 /joint_trajectory_controller/command 话题
└── 映射算法 (SimpleMapping)
    └── 将11维传感器数据映射为6维关节角度
```

## 4. 算法流程设计

### 4.1 主流程
```
1. 节点初始化
   ├── 创建订阅者（订阅传感器数据）
   ├── 创建发布者（发布关节角度）
   └── 初始化映射算法

2. 传感器数据回调
   ├── 接收CSV格式字符串
   ├── 解析为11维浮点数数组
   ├── 验证数据有效性（长度检查）
   └── 调用映射算法

3. 映射算法执行
   ├── 提取关键传感器数据
   ├── 执行映射计算
   └── 返回6个关节角度（弧度）

4. 发布关节角度
   ├── 构造JointTrajectory消息
   ├── 设置关节名称
   ├── 设置关节位置
   └── 发布消息
```

### 4.2 映射算法设计（最简单版本）

#### 方案A：直接映射RPY角度到前3个关节
```
关节1 = Roll角度 × 缩放因子1 (度转弧度)
关节2 = Pitch角度 × 缩放因子2 (度转弧度)
关节3 = Yaw角度 × 缩放因子3 (度转弧度)
关节4 = 0
关节5 = 0
关节6 = 0
```

#### 方案B：使用旋转编码器控制关节1，RPY控制关节2-4
```
关节1 = 旋转编码器角度 × 缩放因子 (度转弧度)
关节2 = Roll角度 × 缩放因子 (度转弧度)
关节3 = Pitch角度 × 缩放因子 (度转弧度)
关节4 = Yaw角度 × 缩放因子 (度转弧度)
关节5 = 0
关节6 = 0
```

#### 方案C：多传感器简单组合（推荐）
```
关节1 = 旋转编码器角度 × scale_encoder (度转弧度)
关节2 = Roll角度 × scale_rpy (度转弧度)
关节3 = Pitch角度 × scale_rpy (度转弧度)
关节4 = Yaw角度 × scale_rpy (度转弧度)
关节5 = 加速度X轴余弦值 × scale_accel × π/2 (映射到±90°)
关节6 = 陀螺仪Z轴角速度 × scale_gyro (归一化后映射)
```

**推荐使用方案B**，因为：
- 简单直观
- 充分利用了主要传感器数据（编码器+RPY）
- 关节4-6设为0，避免过度复杂

### 4.3 映射公式详细设计（方案B）

```python
# 输入：sensor_data[11] (11维数组)
# 输出：joint_angles[6] (6个关节角度，单位：弧度)

# 数据提取
encoder_angle_deg = sensor_data[1]      # 旋转编码器（度）
roll_deg = sensor_data[2]               # Roll角度（度）
pitch_deg = sensor_data[3]               # Pitch角度（度）
yaw_deg = sensor_data[4]                # Yaw角度（度）

# 缩放因子（可配置参数）
scale_encoder = 0.1    # 编码器缩放：编码器每18度对应关节1的0.1弧度
scale_rpy = 0.5        # RPY缩放：传感器角度1度对应关节0.5度

# 映射计算
joint_angles[0] = encoder_angle_deg * scale_encoder * (π/180)  # 度转弧度
joint_angles[1] = roll_deg * scale_rpy * (π/180)
joint_angles[2] = pitch_deg * scale_rpy * (π/180)
joint_angles[3] = yaw_deg * scale_rpy * (π/180)
joint_angles[4] = 0.0
joint_angles[5] = 0.0
```

### 4.4 参数配置
- `scale_encoder`: 编码器缩放因子（默认0.1）
- `scale_rpy`: RPY角度缩放因子（默认0.5）
- `sensor_topic`: 传感器话题名称（默认`/esp/wifi/data`）
- `joint_topic`: 关节控制话题名称（默认`/joint_trajectory_controller/command`）

## 5. 错误处理

### 5.1 数据验证
- 检查CSV字符串是否为空
- 检查解析后的数组长度是否为11
- 检查数值是否为有效浮点数

### 5.2 异常处理
- 数据解析失败：记录错误日志，跳过本次数据
- 映射计算异常：记录错误日志，使用上一次有效值或默认值（全0）

## 6. 运行流程

### 6.1 启动步骤
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

### 6.2 验证方法
- 使用`ros2 topic echo /joint_trajectory_controller/command`查看发布的关节角度
- 使用`ros2 topic echo /esp/wifi/data`查看传感器数据
- 在RViz中观察机械臂运动

## 7. 代码结构预览

### 7.1 simple_mapper_node.py (主节点)
```python
class SimpleMapperNode(Node):
    - __init__(): 初始化订阅者、发布者、映射算法
    - sensor_callback(): 传感器数据回调函数
    - _parse_sensor_data(): 解析CSV字符串
    - _publish_joint_angles(): 发布关节角度
```

### 7.2 simple_mapping.py (映射算法)
```python
class SimpleMapping:
    - __init__(): 初始化缩放因子参数
    - map(): 执行映射计算
    - _deg_to_rad(): 度转弧度辅助函数
```

## 8. 后续扩展方向

虽然当前版本保持简单，但为后续扩展预留了接口：
- 可以添加数据滤波模块
- 可以添加关节限位检查
- 可以添加更复杂的映射策略
- 可以添加参数配置文件支持

---

**设计完成时间**: 2024年
**设计原则**: 简单、可运行、可扩展

