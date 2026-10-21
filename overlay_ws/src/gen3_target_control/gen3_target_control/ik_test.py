import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import MoveItErrorCodes


class IKTest(Node):
    def __init__(self):
        super().__init__("ik_test")
        self.client = self.create_client(
            GetPositionIK, "/compute_ik"
        )

    def run(self):
        if not self.client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("IK 服务不可用")
            return

        request = GetPositionIK.Request()
        ik = request.ik_request

        ik.group_name = "manipulator"
        ik.ik_link_name = "end_effector_link"

        ik.pose_stamped.header.frame_id = "base_link"

        ik.pose_stamped.pose.position.x = 0.343
        ik.pose_stamped.pose.position.y = -0.025
        ik.pose_stamped.pose.position.z = 0.544

        ik.pose_stamped.pose.orientation.x = 0.0
        ik.pose_stamped.pose.orientation.y = 0.997
        ik.pose_stamped.pose.orientation.z = 0.0
        ik.pose_stamped.pose.orientation.w = 0.077

        ik.robot_state.is_diff = True
        ik.avoid_collisions = True
        ik.timeout.sec = 2

        self.get_logger().info("发送末端位姿，开始 IK 求解")

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        result = future.result()
        if result is None:
            self.get_logger().error("IK 服务没有返回结果")
            return

        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(
                f"IK 失败，错误码：{result.error_code.val}"
            )
            return

        joint_state = result.solution.joint_state
        angles = dict(zip(joint_state.name, joint_state.position))

        self.get_logger().info("IK 求解成功！")

        for i in range(1, 8):
            name = f"joint_{i}"
            if name in angles:
                self.get_logger().info(
                    f"{name}: {angles[name]:.6f} rad"
                )


def main():
    rclpy.init()
    node = IKTest()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()