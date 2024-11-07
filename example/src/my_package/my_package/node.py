import cowsay
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class CowsayerNode(Node):
    """Sends cowsay text to a topic regularly"""

    def __init__(self):
        super().__init__("cowsayer")
        self.publisher_ = self.create_publisher(String, "topic", 10)
        self.timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        msg = String(
            data=cowsay.get_output_string("cow", "Hello world")
        )
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: {msg.data}")


def main(args=None):
    rclpy.init(args=args)

    cowsayer = CowsayerNode()

    rclpy.spin(cowsayer)

    cowsayer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
