import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random
import time


class SimPublisher(Node):
    def __init__(self):
        super().__init__('sim_esp_wifi_publisher')

        self.declare_parameter('hz', 2.0)
        self.declare_parameter('topic', 'esp/wifi/data')

        self.hz = float(self.get_parameter('hz').get_parameter_value().double_value)
        self.topic = self.get_parameter('topic').get_parameter_value().string_value
        self.period = 1.0 / max(self.hz, 0.1)

        self.pub = self.create_publisher(String, self.topic, 10)

        self.start_wall = time.time()

        # data[1] before t=7s and =0 afther then
        self.d1_value = 0.0
        self.d1_frozen = False

        self.timer = self.create_timer(self.period, self.timer_cb)
        self.get_logger().info(f"SIM publishing to '{self.topic}' @ {self.hz:.1f} Hz for 20s")

    def piecewise_values(self, t: float):
        # data[0]=1  before t=7s and =0 afther then
        d0 = 1.0 if t < 7.0 else 0.0

        # data[1]：  
        # 0~7s: +18 per tick
        # 7~17s: -18 per tick
        # >=17s: stop）
        if not self.d1_frozen:
            if t < 7.0:
                self.d1_value += 18.0
            elif t < 17.0:
                self.d1_value -= 18.0
            else:
                self.d1_frozen = True  

        d1 = self.d1_value

        # data[2,3,4]
        if t < 7.0:
            d2, d3, d4 = 0.0, 20.0, 0.0
        elif t < 14.0:
            d2, d3, d4 = 30.0, 50.0, 10.0
        else:
            d2, d3, d4 = 70.0, 0.0, 80.0

        # data[5,6,7]
        if t < 7.0:
            d5, d6, d7 = 0.0, 1.0, 0.0
        elif t < 14.0:
            d5, d6, d7 = 1.0, 0.0, 0.0
        else:
            d5, d6, d7 = 0.0, 0.0, -1.0

        # data[8,9,10]
        if t < 7.0:
            d8, d9, d10 = 0.0, 1.0, 0.0
        elif t < 14.0:
            d8, d9, d10 = 1.0, 0.0, 0.0
        else:
            d8, d9, d10 = 0.0, 0.0, -1.0

        return [d0, d1, d2, d3, d4, d5, d6, d7, d8, d9, d10]

    @staticmethod
    def add_noise(vec):
        # Add ±0.1 noise to data[2] - data[10]
        out = vec[:]
        for i in range(8, 11):
            out[i] =out[i]*100 + random.uniform(-20, 20)
        return out

    @staticmethod
    def to_csv(vec):
        return ",".join(f"{x:.4f}" for x in vec)

    def timer_cb(self):
        t = time.time() - self.start_wall

        # exit after 20 seconds
        if t >= 20.0:
            self.get_logger().info("SIM finished (20s). Shutting down.")
            rclpy.shutdown()
            return

        data = self.piecewise_values(t)
        data = self.add_noise(data)

        msg = String()
        msg.data = self.to_csv(data)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = SimPublisher()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == '__main__':
    main()
