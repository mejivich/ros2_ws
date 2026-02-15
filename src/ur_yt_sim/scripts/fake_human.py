#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import math

class FakeHuman(Node):
    def __init__(self):
        super().__init__('fake_human')
        self.publisher = self.create_publisher(Point, '/humans/bodies/body_0/position', 10)
        self.timer = self.create_timer(0.2, self.timer_callback)
        self.angle = 0.0
        self.get_logger().info('Publishing fake human positions...')

    def timer_callback(self):
        msg = Point()

        # Make the "human" move closer/farther to the robot (0.5–1.5 m)
        msg.x = 1.0 + 0.5 * math.sin(self.angle)
        msg.y = 0.0
        msg.z = 0.0

        self.publisher.publish(msg)
        self.get_logger().info(f'Published fake human at x={msg.x:.2f} m')
        self.angle += 0.1

def main(args=None):
    rclpy.init(args=args)
    node = FakeHuman()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
