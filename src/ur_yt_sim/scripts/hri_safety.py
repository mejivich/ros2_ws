#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_srvs.srv import Empty
import math
import time

class HRISafetyMonitor(Node):
    def __init__(self):
        super().__init__('hri_safety')

        self.human_sub = self.create_subscription(Point, '/humans/bodies/body_0/position', self.human_callback, 10)
        self.distance_threshold = 0.7
        self.too_close = False

        # Try both possible stop services
        possible_services = ['/servo_node/stop_servo', '/stop_trajectory_execution']
        self.cli = None
        self.service_name = None
        for service in possible_services:
            cli = self.create_client(Empty, service)
            if cli.wait_for_service(timeout_sec=5.0):
                self.cli = cli
                self.service_name = service
                self.get_logger().info(f'✅ Connected to {service} for stopping motion.')
                break

        if not self.cli:
            self.get_logger().error('No stop service found! Falling back to emergency stop publisher.')
            self.cmd_pub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)

        self.get_logger().info('HRI Safety Monitor started. Waiting for human proximity...')

    def human_callback(self, msg):
        distance = math.sqrt(msg.x**2 + msg.y**2 + msg.z**2)

        if distance < self.distance_threshold and not self.too_close:
            self.too_close = True
            self.get_logger().warn(f'Human too close! Distance={distance:.2f} m → Stopping robot.')
            self.stop_robot()
        elif distance >= self.distance_threshold and self.too_close:
            self.too_close = False
            self.get_logger().info(f'Human moved away (distance={distance:.2f} m). Resuming normal operation.')

    def stop_robot(self):
        if self.cli:
            try:
                future = self.cli.call_async(Empty.Request())
                rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
                if future.done():
                    self.get_logger().info('🛑 Robot motion successfully stopped.')
                else:
                    self.get_logger().warn('Stop service call timed out.')
            except Exception as e:
                self.get_logger().error(f'Stop service failed: {str(e)}')
        else:
            # Emergency fallback
            traj = JointTrajectory()
            traj.joint_names = [
                'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
            ]
            point = JointTrajectoryPoint()
            point.positions = [0.0] * 6
            point.time_from_start = Duration(sec=0)
            traj.points.append(point)
            self.cmd_pub.publish(traj)
            self.get_logger().warn('⚠️ Emergency stop trajectory sent!')

def main(args=None):
    rclpy.init(args=args)
    node = HRISafetyMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
