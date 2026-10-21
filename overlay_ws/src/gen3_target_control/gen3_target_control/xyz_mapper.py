
import json
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped


class XYZMapper(Node):

    def __init__(self):
        super().__init__("xyz_mapper")

        # 初始目标位置（单位：米）
        self.x = 0.343
        self.y = -0.025
        self.z = 0.544

        # 移动速度：2 cm/s
        self.speed = 0.02

        # 工作空间边界（初期仿真测试用）
        self.x_min, self.x_max = -0.5, 0.7
        self.y_min, self.y_max = -0.5, 0.5
        self.z_min, self.z_max = 0.2, 1.0

        # 三轴方向
        self.dx = 0.0
        self.dy = 0.0
        self.dz = 0.0

        # 运动使能
        self.enabled = False

        # WiFi 超时保护
        self.timeout = 0.5
        self.last_receive = None

        # 时间积分
        self.last_update = time.monotonic()

        # 是否存在尚未发布的新坐标
        self.dirty = False

        # 接收 WiFi 数据
        self.sub = self.create_subscription(
            String,
            "/esp/wifi/data",
            self.wifi_callback,
            10
        )

        # 发布三维目标
        self.pub = self.create_publisher(
            PoseStamped,
            "/gen3/target_pose",
            10
        )

        # 50Hz 位置积分
        self.create_timer(0.02, self.update_position)

        # 5Hz 目标发布
        self.create_timer(0.2, self.publish_target)

        self.get_logger().info("XYZ Mapper 已启动")

    def wifi_callback(self, msg):
        try:
            packet = json.loads(msg.data)
            data = packet["data"]

            if len(data) != 11:
                raise ValueError("需要11维数据")

            values = [float(v) for v in data]

            if not all(math.isfinite(v) for v in values):
                raise ValueError("存在非法数字")

            # 按键方向 -1 / 0 / 1
            self.dx = max(-1.0, min(1.0, values[0]))
            self.dy = max(-1.0, min(1.0, values[1]))
            self.dz = max(-1.0, min(1.0, values[2]))

            # data[3]：1允许，0停止积分
            self.enabled = values[3] >= 0.5

            self.last_receive = time.monotonic()

            if not self.enabled:
                self.dx = self.dy = self.dz = 0.0

        except (ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
            self.enabled = False
            self.dx = self.dy = self.dz = 0.0
            self.get_logger().warning(f"WiFi数据错误：{exc}")

    def update_position(self):
        now = time.monotonic()
        dt = min(now - self.last_update, 0.1)
        self.last_update = now

        # 数据断开：停止位置积分
        if (
            self.last_receive is None
            or now - self.last_receive > self.timeout
        ):
            self.enabled = False
            self.dx = self.dy = self.dz = 0.0
            return

        if not self.enabled:
            return

        old_position = (self.x, self.y, self.z)

        # 归一化斜向输入，避免斜向移动更快
        length = math.sqrt(
            self.dx**2 + self.dy**2 + self.dz**2
        )

        if length > 1.0:
            vx = self.dx / length
            vy = self.dy / length
            vz = self.dz / length
        else:
            vx, vy, vz = self.dx, self.dy, self.dz

        # 按真实经过时间累积距离
        self.x += vx * self.speed * dt
        self.y += vy * self.speed * dt
        self.z += vz * self.speed * dt

        # 工作空间限幅
        self.x = max(self.x_min, min(self.x, self.x_max))
        self.y = max(self.y_min, min(self.y, self.y_max))
        self.z = max(self.z_min, min(self.z, self.z_max))

        if (self.x, self.y, self.z) != old_position:
            self.dirty = True

    def publish_target(self):
        # 只在位置变化且输入有效时发布
        if not self.enabled or not self.dirty:
            return

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        msg.pose.position.x = self.x
        msg.pose.position.y = self.y
        msg.pose.position.z = self.z

        # 固定姿态由 target_controller 负责
        msg.pose.orientation.w = 1.0

        self.pub.publish(msg)
        self.dirty = False

        self.get_logger().info(
            f"Target XYZ: "
            f"{self.x:.4f}, "
            f"{self.y:.4f}, "
            f"{self.z:.4f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = XYZMapper()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
