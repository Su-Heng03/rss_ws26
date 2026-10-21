"""
Simple sensor data to joint angle mapping algorithm module
Implementation Strategy B: Encoder + RPY angle mapping
Implementation Strategy D: Vector tracking (using acceleration cosine values)
"""

import math
from typing import List


class SimpleMapping:
    """Simple sensor data to joint angle mapping class"""
    
    def __init__(self, scale_encoder: float = 0.1, scale_rpy: float = 0.5):
        """
        Initialize mapping algorithm
        
        Args:
            scale_encoder: Encoder scaling factor (default 0.1)
            scale_rpy: RPY angle scaling factor (default 0.5)
        """
        self.scale_encoder = scale_encoder
        self.scale_rpy = scale_rpy
        self.pi = math.pi
    
    def map(self, sensor_data: List[float], mapping_strategy: str = 'B') -> List[float]:
        """
        Map sensor data to joint angles
        
        Args:
            sensor_data: 11-dimensional sensor data array
                [Infrared, Encoder, Roll, Pitch, Yaw, 
                AccelX, AccelY, AccelZ, 
                GyroX, GyroY, GyroZ]
            mapping_strategy: Mapping strategy, 'B' for Strategy B, 'D' for Strategy D
        
        Returns:
            7 joint angles (radians)
        """
        if mapping_strategy == 'B':
            angles = self._map_strategy_b(sensor_data)
        elif mapping_strategy == 'D':
            angles = self._map_strategy_d(sensor_data)
        else:
            raise ValueError(f"Unknown mapping strategy: {mapping_strategy}, supports 'B' or 'D'")
            
        return angles
    
    def _map_strategy_b(self, sensor_data: List[float]) -> List[float]:
        """
        Strategy B: Encoder + RPY angle mapping
        
        Mapping rules:
        - Joint 1 = Encoder angle * scale_encoder * (pi/180)
        - Joint 2 = Roll angle * scale_rpy * (pi/180)
        - Joint 3 = Pitch angle * scale_rpy * (pi/180)
        - Joint 4 = Yaw angle * scale_rpy * (pi/180)
        - Joint 5 = 0.0
        - Joint 6 = 0.0
        - Joint 7 = 0.0
        
        Args:
            sensor_data: 11-dimensional sensor data array
        
        Returns:
            7 joint angles (radians)
        """
        # Extract key sensor data
        encoder_angle_deg = sensor_data[1]  # Rotary encoder (degrees)
        roll_deg = sensor_data[2]            # Roll angle (degrees)
        pitch_deg = sensor_data[3]           # Pitch angle (degrees)
        yaw_deg = sensor_data[4]             # Yaw angle (degrees)
        
        # Degree to radian conversion factor
        deg_to_rad = self.pi / 180.0
        
        # Execute mapping calculation (extended to 7 DOF, adapted for Kinova Gen3)
        joint_angles = [
            encoder_angle_deg * self.scale_encoder * deg_to_rad,  # Joint 1
            roll_deg * self.scale_rpy * deg_to_rad,               # Joint 2
            pitch_deg * self.scale_rpy * deg_to_rad,              # Joint 3
            yaw_deg * self.scale_rpy * deg_to_rad,                # Joint 4
            0.0,                                                  # Joint 5
            0.0,                                                  # Joint 6
            0.0                                                   # Joint 7
        ]
        
        return joint_angles
    
    def _map_strategy_d(self, sensor_data: List[float]) -> List[float]:
        """
        Strategy D: Vector tracking mapping
        
        Use the 5th, 6th, 7th dimensions of sensor data (acceleration cosine values) as input vectors X, Y, Z.
        Rotate the first joint (J1) and second joint (J2) to align the end effector's Z-axis
        with the given vector direction.
        
        Mapping rules:
        1. Input: 3D vector x, y, z (from data[5], data[6], data[7])
        2. Validity check: if x=y=z=0, return error
        3. Calculate q1: use math.atan2(y, x)
        4. Calculate q2: use math.atan2(sqrt(x*x + y*y), z)
        5. Output: Target joint angle sequence [q1, q2, 0, 0, 0, 0, 0]
        
        Args:
            sensor_data: 11-dimensional sensor data array
        
        Returns:
            7 joint angles (radians)
        
        Raises:
            ValueError: If input vector is zero vector
        """
        # Extract vector components (5th, 6th, 7th dimensions: Accel X, Y, Z cosine values)
        w = sensor_data[1]  # Rotation controller data
        x = sensor_data[7]  # Accel_X cosine
        y = sensor_data[6]  # Accel_Y cosine
        z = sensor_data[5]  # Accel_Z cosine

        temp = (w * 2 * 3.14) / 360

        # Validity check: if x=y=z=0, return error
        norm = math.sqrt(x**2 + y**2 + z**2 + w**2)
        if norm < 1e-6:
            raise ValueError("Input vector is zero vector, cannot determine direction")
        
        # Calculate q1 (1st joint, Yaw, horizontal yaw)
        # atan2(y, x) Returns range [-pi, pi]
        q1 = math.atan2(y, x)
        
        # Calculate q2 (2nd joint, Pitch, vertical pitch)
        # Calculate horizontal component length
        horizontal_dist = math.sqrt(x**2 + y**2)
        # atan2(horizontal, vertical) Get angle with Z axis
        q2 = math.atan2(horizontal_dist, z)
        
        q4 =  1.57 - q2 -temp

        q6 = 1.57 + temp
        # Output target joint angle sequence
        joint_angles = [
            float(q1),   # Joint 1
            float(q2),   # Joint 2
            0.0,  # Joint 3
            float(q4),  # Joint 4
            0.0,  # Joint 5
            float(q6),  # Joint 6
            0.0   # Joint 7
        ]
        
        return joint_angles
    
    def set_scale_encoder(self, scale: float):
        """Set encoder scaling factor"""
        self.scale_encoder = scale
    
    def set_scale_rpy(self, scale: float):
        """Set RPY angle scaling factor"""
        self.scale_rpy = scale
