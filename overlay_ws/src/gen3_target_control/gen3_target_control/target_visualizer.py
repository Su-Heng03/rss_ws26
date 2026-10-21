
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker


class TargetVisualizer(Node):

    def __init__(self):
        super().__init__("target_visualizer")

        # 订阅 Mapper 发布的目标
        self.sub = self.create_subscription(
            PoseStamped,
            "/gen3/target_pose",
            self.target_callback,
            10
        )

        # 向 RViz 发布 Marker
        self.pub = self.create_publisher(
            Marker,
            "/gen3/target_marker",
            10
        )

        self.get_logger().info(
            "Target Visualizer 已启动，等待目标坐标"
        )

    def target_callback(self, msg):

        marker = Marker()

        # 坐标系与目标一致
        marker.header.frame_id = (
            msg.header.frame_id or "base_link"
        )
        marker.header.stamp = self.get_clock().now().to_msg()

        # 固定 ID：每次更新同一个小球
        marker.ns = "gen3_target"
        marker.id = 0

        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        # XYZ
        #marker.pose.position.x = msg.pose.position.x
        #marker.pose.position.y = msg.pose.position.y
        #marker.pose.position.z = msg.pose.position.z

        # 球体无需使用目标四元数
        #marker.pose.orientation.w = 1.0
        marker.pose = msg.pose
        # 球体直径 3 cm
        marker.scale.x = 0.06
        marker.scale.y = 0.03
        marker.scale.z = 0.02

        # 红色
        marker.color.r = 1.0
        marker.color.g = 0.1
        marker.color.b = 0.1
        marker.color.a = 1.0

        # 0 表示持续存在，直到更新或删除
        marker.lifetime.sec = 0

        self.pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)

    node = TargetVisualizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
