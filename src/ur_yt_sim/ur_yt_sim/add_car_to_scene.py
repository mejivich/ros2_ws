#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit_commander import PlanningSceneInterface
from moveit_commander.robot_commander import RobotCommander
from geometry_msgs.msg import Pose

class AddCarToScene(Node):
    def __init__(self):
        super().__init__('add_car_to_scene')
        self.scene = PlanningSceneInterface()
        self.robot = RobotCommander()
        self.get_logger().info("Waiting for Planning Scene to be ready...")
        self.create_timer(2.0, self.add_car)

    def add_car(self):
        # Define car pose
        car_pose = Pose()
        car_pose.position.x = 1.0  # adjust to where car is in Gazebo
        car_pose.position.y = 0.0
        car_pose.position.z = 0.0
        car_pose.orientation.w = 1.0

        # Add car as a mesh (URDF)
        car_urdf_path = "/root/ros2_ws/install/ur_yt_sim/share/ur_yt_sim/urdf/car_with_inlet.urdf"
        self.scene.add_mesh("car", car_pose, car_urdf_path)
        self.get_logger().info("Car added to MoveIt planning scene.")

        # Stop after adding once
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = AddCarToScene()
    rclpy.spin(node)

if __name__ == "__main__":
    main()
