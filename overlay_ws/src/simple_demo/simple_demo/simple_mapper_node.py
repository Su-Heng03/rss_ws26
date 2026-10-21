"""
Sensor data to joint angle mapping node
Main node: Subscribes to sensor data, executes mapping algorithm, publishes joint angles
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from .simple_mapping import SimpleMapping


class SimpleMapperNode(Node):
    """Simple sensor to joint angle mapping node"""
    
    def __init__(self):
        """Initialize node"""
        super().__init__('simple_mapper_node')
        
        # Declare parameters
        self.declare_parameter('sensor_topic', '/esp/wifi/data')
        self.declare_parameter('joint_topic', '/joint_trajectory_controller/joint_trajectory')
        self.declare_parameter('scale_encoder', 0.1)
        self.declare_parameter('scale_rpy', 0.5)
        self.declare_parameter('mapping_strategy', 'B')  # 'B' or 'D'
        self.declare_parameter('smoothing_factor', 0.1)  # Smoothing factor (0.0 ~ 1.0), smaller is smoother
        self.declare_parameter('max_velocity', 0.6)     # Max velocity limit (rad/frame), prevents sudden changes
        
        # Get parameters
        sensor_topic = self.get_parameter('sensor_topic').get_parameter_value().string_value
        joint_topic = self.get_parameter('joint_topic').get_parameter_value().string_value
        scale_encoder = self.get_parameter('scale_encoder').get_parameter_value().double_value
        scale_rpy = self.get_parameter('scale_rpy').get_parameter_value().double_value
        mapping_strategy = self.get_parameter('mapping_strategy').get_parameter_value().string_value
        self.smoothing_factor = self.get_parameter('smoothing_factor').get_parameter_value().double_value
        self.max_velocity = self.get_parameter('max_velocity').get_parameter_value().double_value
        
        # Validate mapping strategy
        if mapping_strategy not in ['B', 'D']:
            self.get_logger().warn(
                f'Invalid mapping strategy: {mapping_strategy}, using default strategy B'
            )
            mapping_strategy = 'B'
        
        self.mapping_strategy = mapping_strategy
        
        # Initialize mapping algorithm
        self.mapping = SimpleMapping(
            scale_encoder=scale_encoder,
            scale_rpy=scale_rpy
        )
        
        # Subscribe to sensor data topic
        self.sensor_subscription = self.create_subscription(
            String,
            sensor_topic,
            self.sensor_callback,
            10
        )
        
        # Publish joint angles topic
        self.joint_publisher = self.create_publisher(
            JointTrajectory,
            joint_topic,
            10
        )
        
        # Joint names (Kinova Gen3 is 7 DOF)
        self.joint_names = [
            'joint_1',
            'joint_2',
            'joint_3',
            'joint_4',
            'joint_5',
            'joint_6',
            'joint_7'
        ]
        
        # Statistics
        self.message_count = 0
        self.error_count = 0
        self.last_joint_angles = None  # Stores previous frame angles for smoothing
        
        self.get_logger().info('Simple mapping node started')
        self.get_logger().info(f'Subscribed to topic: {sensor_topic}')
        self.get_logger().info(f'Publishing to topic: {joint_topic}')
        self.get_logger().info(f'Mapping strategy: {mapping_strategy} (B=Encoder+RPY, D=Vector Tracking)')
        if mapping_strategy == 'B':
            self.get_logger().info(f'Encoder scaling factor: {scale_encoder}')
            self.get_logger().info(f'RPY scaling factor: {scale_rpy}')
        elif mapping_strategy == 'D':
            self.get_logger().info('Using acceleration cosine values (data[5], data[6], data[7]) as input vector')
    
    def sensor_callback(self, msg: String):
        """
        Sensor data callback function
        
        Args:
            msg: Sensor data message (CSV format string)
        """
        self.message_count += 1
        
        try:
            # Parse CSV string
            data_list = self._parse_sensor_data(msg.data)
            
            # Validate data length
            if len(data_list) != 11:
                self.get_logger().error(
                    f'Data length error: expected 11, actual {len(data_list)}'
                )
                self.error_count += 1
                return
            
            # Check infrared sensor data (data[0])
            # If 1 (greater than 0.5), stop action, do not map or publish
            if data_list[0] > 0.5:
                if self.message_count % 50 == 0:
                    self.get_logger().info(f'Infrared sensor triggered (value {data_list[0]}), arm stopping')
                return
            
            # Execute mapping calculation
            target_joint_angles = self.mapping.map(data_list, mapping_strategy=self.mapping_strategy)
            
            # Low-pass filter smoothing (Exponential Moving Average)
            if self.last_joint_angles is None:
                joint_angles = target_joint_angles
            else:
                alpha = self.smoothing_factor
                max_vel = self.max_velocity
                joint_angles = []
                for t, l in zip(target_joint_angles, self.last_joint_angles):
                    # 1. Calculate expected step (EMA)
                    step = alpha * (t - l)
                    # 2. Velocity limiting (Clamping): limit max change per frame
                    if step > max_vel: step = max_vel
                    elif step < -max_vel: step = -max_vel
                    
                    joint_angles.append(l + step)
            self.last_joint_angles = joint_angles
            
            # Publish joint angles
            self._publish_joint_angles(joint_angles)
            
        except ValueError as e:
            self.get_logger().error(f'Data parsing error: {e}')
            self.error_count += 1
        except Exception as e:
            self.get_logger().error(f'Error processing sensor data: {e}')
            self.error_count += 1
    
    def _parse_sensor_data(self, csv_string: str) -> list:
        """
        Parse CSV format sensor data string
        
        Args:
            csv_string: CSV format string, e.g. "1.0,18.0,0.0,20.0,..."
        
        Returns:
            11-dimensional float array
        """
        data_str = csv_string.strip()
        if not data_str:
            raise ValueError('Sensor data string is empty')
        
        try:
            data_list = [float(x.strip()) for x in data_str.split(',')]
            return data_list
        except ValueError as e:
            raise ValueError(f'Cannot parse sensor data: {e}')
    
    def _publish_joint_angles(self, joint_angles: list):
        """
        Publish joint angles
        
        Args:
            joint_angles: 7 joint angles (radians)
        """

        try:
            # If joint_angles is a numpy array, flatten first, then convert
            import numpy as np
            if hasattr(joint_angles, 'flatten'):
                 clean_angles = [float(x) for x in joint_angles.flatten()]
            else:
                 clean_angles = [float(x) for x in joint_angles]
        except Exception as e:
            self.get_logger().error(f"Data conversion failed: {e}, raw data: {joint_angles}")
            return
            
        # Create JointTrajectory message
        trajectory_msg = JointTrajectory()
        trajectory_msg.header.stamp = self.get_clock().now().to_msg()
        trajectory_msg.header.frame_id = 'base_link'
        trajectory_msg.joint_names = self.joint_names
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = joint_angles
        point.velocities = [0.0] * 7  # Velocity set to 0, calculated by controller
        point.accelerations = [0.0] * 7  # Acceleration set to 0
        point.time_from_start = Duration(sec=0, nanosec=100000000)  # Set time to reach target to 0.1 seconds
        
        trajectory_msg.points = [point]
        
        # Publish message
        self.joint_publisher.publish(trajectory_msg)
        
        # Periodically output statistics
        if self.message_count % 50 == 0:
            self.get_logger().info(
                f'Processed {self.message_count} messages, {self.error_count} errors'
            )
            self.get_logger().debug(
                f'Current joint angles: {[f"{x:.4f}" for x in joint_angles]}'
            )


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    node = SimpleMapperNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Node is shutting down...')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
