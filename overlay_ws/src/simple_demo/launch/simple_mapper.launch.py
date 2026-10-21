"""
启动simple_mapper_node的launch文件
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """生成launch描述"""
    
    # 声明launch参数
    sensor_topic_arg = DeclareLaunchArgument(
        'sensor_topic',
        default_value='/esp/wifi/data',
        description='传感器数据话题名称'
    )
    
    joint_topic_arg = DeclareLaunchArgument(
        'joint_topic',
        default_value='/joint_trajectory_controller/command',
        description='关节控制话题名称'
    )
    
    scale_encoder_arg = DeclareLaunchArgument(
        'scale_encoder',
        default_value='0.1',
        description='编码器缩放因子'
    )
    
    scale_rpy_arg = DeclareLaunchArgument(
        'scale_rpy',
        default_value='0.5',
        description='RPY角度缩放因子'
    )
    
    # 创建节点
    mapper_node = Node(
        package='simple_demo',
        executable='simple_mapper_node',
        name='simple_mapper_node',
        parameters=[{
            'sensor_topic': LaunchConfiguration('sensor_topic'),
            'joint_topic': LaunchConfiguration('joint_topic'),
            'scale_encoder': LaunchConfiguration('scale_encoder'),
            'scale_rpy': LaunchConfiguration('scale_rpy'),
        }],
        output='screen'
    )
    
    return LaunchDescription([
        sensor_topic_arg,
        joint_topic_arg,
        scale_encoder_arg,
        scale_rpy_arg,
        mapper_node,
    ])

