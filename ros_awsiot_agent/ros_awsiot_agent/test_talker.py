#!/usr/bin/env python3
# license removed for brevity
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.pub = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.cnt = 0

    def timer_callback(self):
        msg = String()
        hello_str = "Hello AWS IoT, greetings from ROS!"
        self.cnt += 1
        msg.data = hello_str + f" (count: {self.cnt})"
        self.get_logger().info(f'Publishing: {hello_str} (count: {self.cnt})')
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Talker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
