#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PointStamped
from hri_msgs.msg import IdsList

class GazeboHRIBridge(Node):
    def __init__(self):
        super().__init__('gazebo_hri_bridge')
        self.get_logger().info('🤖 Gazebo-HRI bridge started.')

        # Subscribe to Gazebo model states
        self.create_subscription(ModelStates, '/gazebo/model_states', self.model_callback, 10)

        # Publisher for human IDs
        self.ids_pub = self.create_publisher(IdsList, '/humans/bodies', 10)

        # Dynamic publishers for each detected human
        self.human_publishers = {}

        # The prefix used for human models in Gazebo (e.g., "human1", "human2")
        self.human_prefix = 'human'

    def model_callback(self, msg: ModelStates):
        ids = []
        for name, pose in zip(msg.name, msg.pose):
            if self.human_prefix in name.lower():  # match 'human', 'Human1', etc.
                ids.append(name)

                # Create a new publisher if needed
                if name not in self.human_publishers:
                    topic = f'/humans/bodies/{name}/position'
                    self.human_publishers[name] = self.create_publisher(PointStamped, topic, 10)
                    self.get_logger().info(f'🧍 Added HRI publisher for: {topic}')

                # Publish position as PointStamped
                point_msg = PointStamped()
                point_msg.header.stamp = self.get_clock().now().to_msg()
                point_msg.header.frame_id = name  # unique ID for each human
                point_msg.point.x = pose.position.x
                point_msg.point.y = pose.position.y
                point_msg.point.z = pose.position.z
                self.human_publishers[name].publish(point_msg)

        # Publish list of active humans
        ids_msg = IdsList()
        ids_msg.ids = ids
        self.ids_pub.publish(ids_msg)

        if not ids:
            self.get_logger().warn_once('⚠️ No human models found in Gazebo.')

def main(args=None):
    rclpy.init(args=args)
    node = GazeboHRIBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()