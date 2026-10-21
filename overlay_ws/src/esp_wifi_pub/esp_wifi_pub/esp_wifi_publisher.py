import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket


class EspWifiPublisher(Node):
    def __init__(self):
        super().__init__('esp_wifi_publisher')

        self.declare_parameter('host', '192.168.1.11')
        self.declare_parameter('port', 80)
        self.declare_parameter('path', '/data')
        self.declare_parameter('hz', 10.0)

        self.host = self.get_parameter('host').get_parameter_value().string_value
        self.port = int(self.get_parameter('port').get_parameter_value().integer_value)
        self.path = self.get_parameter('path').get_parameter_value().string_value
        hz = float(self.get_parameter('hz').get_parameter_value().double_value)

        self.pub = self.create_publisher(String, 'esp/wifi/data', 10)
        self.timer = self.create_timer(1.0 / max(hz, 0.1), self.timer_cb)

        self.timeout = 0.5
        self.get_logger().info(f"Fetching: http://{self.host}:{self.port}{self.path} @ {hz:.1f}Hz")

    def http_get_once(self) -> str:
        req = (
            f"GET {self.path} HTTP/1.1\r\n"
            f"Host: {self.host}\r\n"
            f"Connection: close\r\n"
            f"User-Agent: ros2-wsl\r\n"
            f"\r\n"
        ).encode('utf-8')

        with socket.create_connection((self.host, self.port), timeout=self.timeout) as s:
            s.settimeout(self.timeout)
            s.sendall(req)

            chunks = []
            total = 0
            while total < 8192:   #limiting data size
                try:
                    buf = s.recv(1024)
                except socket.timeout:
                    break
                if not buf:
                    break
                chunks.append(buf)
                total += len(buf)

        raw = b"".join(chunks).decode('utf-8', errors='ignore')


        sep = "\r\n\r\n"
        idx = raw.find(sep)
        body = raw[idx + len(sep):] if idx != -1 else raw
        return body.strip()

    def timer_cb(self):
        try:
            text = self.http_get_once()
            if not text:
                self.get_logger().warn("Empty body")
                return

            msg = String()
            msg.data = text
            self.pub.publish(msg)

        except Exception as e:
            self.get_logger().warn(f"Fetch failed: {e}")


def main():
    rclpy.init()
    node = EspWifiPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
