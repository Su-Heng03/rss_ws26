
import json
import math

import numpy as np
import rclpy

from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped


class PoseBridge(Node):

    def __init__(self):
        super().__init__("pose_bridge")

        # ==================================
        # 相机外参：以后主要修改这里
        # 单位：米，参考坐标系 base_link
        # ==================================
        self.camera_x = 0.40
        self.camera_y = 0.00
        self.camera_z = 0.90

        # 相机安装约定：
        # 1. 镜头竖直向下
        # 2. 图像顶部朝向机器人底座
        # 3. base_link: +X前方，+Z向上
        #
        # 相机光学坐标：
        # +X 图像右方
        # +Y 图像下方
        # +Z 镜头前方
        #
        # camera -> base_link 旋转矩阵
        self.R = np.array([
            [0.0,  -1.0,  0.0],
            [-1.0, 0.0,  0.0],
            [0.0,  0.0, -1.0]
        ])

        self.translation = np.array([
            self.camera_x,
            self.camera_y,
            self.camera_z
        ])

        self.sub = self.create_subscription(
            String,
            "/esp/wifi/data",
            self.wifi_callback,
            10
        )

        self.pub = self.create_publisher(
            PoseStamped,
            "/gen3/target_pose",
            10
        )

        self.get_logger().info(
            "Pose Bridge 已启动：Camera -> base_link"
        )

    @staticmethod
    def matrix_to_quaternion(R):
        """
        旋转矩阵 -> 四元数，返回 xyzw
        """
        trace = np.trace(R)

        if trace > 0:
            s = math.sqrt(trace + 1.0) * 2.0
            qw = 0.25 * s
            qx = (R[2, 1] - R[1, 2]) / s
            qy = (R[0, 2] - R[2, 0]) / s
            qz = (R[1, 0] - R[0, 1]) / s
        else:
            i = int(np.argmax(np.diag(R)))
            j = (i + 1) % 3
            k = (i + 2) % 3

            s = math.sqrt(
                1.0 + R[i, i] - R[j, j] - R[k, k]
            ) * 2.0

            q = np.zeros(4)
            q[i] = 0.25 * s
            q[3] = (R[k, j] - R[j, k]) / s
            q[j] = (R[j, i] + R[i, j]) / s
            q[k] = (R[k, i] + R[i, k]) / s

            qx, qy, qz, qw = q

        return np.array([qx, qy, qz, qw])

    @staticmethod
    def quaternion_multiply(a, b):
        """
        四元数相乘 a ⊗ b
        顺序 xyzw
        """
        ax, ay, az, aw = a
        bx, by, bz, bw = b

        return np.array([
            aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw,
            aw*bw - ax*bx - ay*by - az*bz
        ])

    def wifi_callback(self, msg):
        try:
            packet = json.loads(msg.data)
            data = packet["data"]

            if not isinstance(data, list) or len(data) != 11:
                raise ValueError("需要11维数据")

            v = np.array(data, dtype=float)

            if not np.isfinite(v).all():
                raise ValueError("包含非法数值")

            # 相机坐标系下的位置
            p_camera = v[0:3]

            # 相机坐标系下的物体四元数
            q_camera = v[3:7]

            norm = np.linalg.norm(q_camera)

            if norm < 1e-6:
                # 包括目标丢失时发送的11个0
                return

            q_camera = q_camera / norm

            # 1. 相机位置 -> base_link
            p_base = self.R @ p_camera + self.translation

            # 2. 相机姿态 -> base_link
            q_transform = self.matrix_to_quaternion(self.R)

            q_base = self.quaternion_multiply(
                q_transform,
                q_camera
            )

            q_base /= np.linalg.norm(q_base)

            # 3. 生成 ROS2 PoseStamped
            pose = PoseStamped()

            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = "base_link"

            pose.pose.position.x = float(p_base[0])
            pose.pose.position.y = float(p_base[1])
            pose.pose.position.z = float(p_base[2])

            pose.pose.orientation.x = float(q_base[3])
            pose.pose.orientation.y = float(q_base[2])
            pose.pose.orientation.z = -float(q_base[1])
            pose.pose.orientation.w = -float(q_base[0])

            self.pub.publish(pose)

        except (ValueError, KeyError, TypeError) as exc:
            self.get_logger().warning(
                f"位姿转换失败：{exc}"
            )


def main(args=None):
    rclpy.init(args=args)

    node = PoseBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
