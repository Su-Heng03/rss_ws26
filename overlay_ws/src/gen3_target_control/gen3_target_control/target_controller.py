
import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes


class TargetController(Node):
    def __init__(self):
        super().__init__("target_controller")

        self.group = ReentrantCallbackGroup()

        self.ik_client = self.create_client(
            GetPositionIK, "/compute_ik",
            callback_group=self.group
        )
        self.move_client = ActionClient(
            self, MoveGroup, "/move_action",
            callback_group=self.group
        )

        self.latest_target = None
        self.target_version = 0
        self.processed_version = 0
        self.busy = False
        self.lock = threading.Lock()

        self.create_subscription(
            PoseStamped,
            "/gen3/target_pose",
            self.target_callback,
            10,
            callback_group=self.group
        )

        self.create_timer(
            1.0,
            self.dispatch_target,
            callback_group=self.group
        )

        # 第一阶段固定工具姿态：沿用已验证的测试姿态
        norm = math.sqrt(0.997**2 + 0.077**2)
        self.fixed_q = (0.0, 0.997/norm, 0.0, 0.077/norm)

        self.get_logger().info(
            "目标订阅节点已启动：/gen3/target_pose，调度周期 3 秒"
        )

    def target_callback(self, msg):
        if msg.header.frame_id != "base_link":
            self.get_logger().warn("目标坐标系必须为 base_link")
            return

        p = msg.pose.position
        q = msg.pose.orientation

        values = (
            p.x, p.y, p.z,
            q.x, q.y, q.z, q.w
        )

        if not all(math.isfinite(v) for v in values):
            self.get_logger().warn("位姿包含非法数值")
            return

        norm = math.sqrt(
            q.x*q.x + q.y*q.y +
            q.z*q.z + q.w*q.w
        )

        if norm < 1e-6:
            self.get_logger().warn("无效四元数")
            return

        target = (
            p.x, p.y, p.z,
            q.x / norm,
            q.y / norm,
            q.z / norm,
            q.w / norm
        )

        with self.lock:
            self.latest_target = target
            self.target_version += 1


    def dispatch_target(self):
        with self.lock:
            if self.busy or self.latest_target is None:
                return
            if self.target_version == self.processed_version:
                return

            xyz = self.latest_target
            version = self.target_version
            self.busy = True
            self.processed_version = version

        self.get_logger().info(f"开始处理目标 XYZ={xyz}")

        thread = threading.Thread(
            target=self.process_target,
            args=(xyz,),
            daemon=True
        )
        thread.start()

    def process_target(self, xyz):
        try:
            self.plan_and_execute(xyz)
        except Exception as exc:
            self.get_logger().error(f"目标处理异常：{exc}")
        finally:
            with self.lock:
                self.busy = False

    def wait_future(self, future, timeout=60.0):
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        if not event.wait(timeout):
            raise TimeoutError("等待 ROS2 响应超时")
        return future.result()

    def plan_and_execute(self, xyz):
        x, y, z, qx, qy, qz, qw = xyz

        if not self.ik_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("IK 服务不可用")
            return

        request = GetPositionIK.Request()
        ik = request.ik_request

        ik.group_name = "manipulator"
        ik.ik_link_name = "end_effector_link"
        ik.pose_stamped.header.frame_id = "base_link"
        ik.pose_stamped.header.stamp = self.get_clock().now().to_msg()

        p = ik.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = x, y, z
        p.orientation.x = qx
        p.orientation.y = qy
        p.orientation.z = qz
        p.orientation.w = qw

        ik.robot_state.is_diff = True
        ik.avoid_collisions = True
        ik.timeout.sec = 2

        response = self.wait_future(
            self.ik_client.call_async(request),
            timeout=8.0
        )

        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(
                f"IK 求解失败：{response.error_code.val}"
            )
            return

        states = dict(zip(
            response.solution.joint_state.name,
            response.solution.joint_state.position
        ))

        joint_names = [f"joint_{i}" for i in range(1, 8)]

        if any(n not in states for n in joint_names):
            self.get_logger().error("IK 返回关节数据不完整")
            return

        constraints = Constraints()
        for name in joint_names:
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = states[name]
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        if not self.move_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("MoveGroup Action 不可用")
            return

        goal = MoveGroup.Goal()
        goal.request.group_name = "manipulator"
        goal.request.start_state.is_diff = True
        goal.request.pipeline_id = "ompl"
        goal.request.planner_id = "RRTConnectkConfigDefault"
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.1
        goal.request.max_acceleration_scaling_factor = 0.1
        goal.request.goal_constraints.append(constraints)

        # 初次订阅测试先只规划，验证后再开启执行
        goal.planning_options.plan_only = False

        self.get_logger().info("IK 成功，开始 MoveIt 规划")

        handle = self.wait_future(
            self.move_client.send_goal_async(goal),
            timeout=10.0
        )

        if handle is None or not handle.accepted:
            self.get_logger().error("MoveGroup 拒绝请求")
            return

        response = self.wait_future(
            handle.get_result_async(),
            timeout=120.0
        )

        result = response.result

        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(
                f"MoveIt 失败：{result.error_code.val}"
            )
            return

        points = result.planned_trajectory.joint_trajectory.points
        self.get_logger().info(
            f"MoveIt 规划成功，轨迹点数量：{len(points)}"
        )


def main():
    rclpy.init()
    node = TargetController()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
