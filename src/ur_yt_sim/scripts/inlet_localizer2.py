#!/usr/bin/env python3
#imports
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

import numpy as np

from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped, PoseStamped
import sensor_msgs_py.point_cloud2 as pc2

import tf2_ros
import tf2_geometry_msgs  


class Inlet3DFromPointcloud(Node):
    def __init__(self):
        super().__init__("inlet_3d_estimator_simple")
        # κάνει subscribe στο point cloud
        self.sub_pc = self.create_subscription(
            PointCloud2,
            "/camera/points",
            self.pc_cb,
            10,
        )
        # κάνει subscribe στο center_px
        self.sub_bbox = self.create_subscription(
            PointStamped,
            "/inlet/center_px",
            self.bbox_cb,
            10,
        )
        # εκδίδει pose xyz
        self.pub_pose = self.create_publisher(
            PoseStamped,
            "/inlet/preinsert_pose",
            10,
        )

        self.pointcloud_msg = None
        self.done = False
      # παίρνει μια περιοχή γύρω απο το πιξελ  9x9
        self.roi_half = 4        # ROI = (2*roi_half+1)^2 -> 9x9
        self.min_points = 20     # minimum valid points in ROI
        self.lock_after_good = 1 # publish once then stop
        self.good_count = 0

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        #αποθήκευση του point cloud
    def pc_cb(self, msg: PointCloud2):
        if self.done:
            return
        self.pointcloud_msg = msg

    def bbox_cb(self, msg: PointStamped):
        if self.done or self.pointcloud_msg is None:
            return

        u = int(msg.point.x)
        v = int(msg.point.y)

        H = int(self.pointcloud_msg.height)
        W = int(self.pointcloud_msg.width)
        #ελέγχει αν το  point cloud βρίσκεται εντός ορίων
        if not (0 <= u < W and 0 <= v < H):
            self.get_logger().warn(
                f"Pixel out of bounds: u={u}, v={v}, W={W}, H={H}"
            )
            return

        # Διαβάζει το point cloud και το μετατρέπει σε πίνακα numpy (H,W,3)
        try:
            cloud = pc2.read_points_numpy(
                self.pointcloud_msg,
                field_names=("x", "y", "z"),
            )
            cloud = cloud.reshape(H, W, 3)
        except Exception as e:
            self.get_logger().warn(f"PointCloud read/reshape error: {e}")
            return

        # ROI bounds
        u0 = max(0, u - self.roi_half)
        u1 = min(W - 1, u + self.roi_half)
        v0 = max(0, v - self.roi_half)
        v1 = min(H - 1, v + self.roi_half)

        region = cloud[v0:v1 + 1, u0:u1 + 1, :].reshape(-1, 3)
        region = region[np.isfinite(region).all(axis=1)]
        
        # άμα υπάρχουν λιγότερα valid 3D σημεία, βγάζει warning και δεν δημοσιεύει
        if region.shape[0] < self.min_points:
            self.get_logger().warn(
                f"Not enough valid 3D points in ROI: {region.shape[0]}"
            )
            return
        
        #  Εκτίμηση median συντεταγμένων xyz
        xyz = np.median(region, axis=0)
        x, y, z = float(xyz[0]), float(xyz[1]), float(xyz[2])

        if not np.isfinite([x, y, z]).all():
            self.get_logger().warn("Median ROI produced invalid point")
            return

        # Θέση σε PoseStamped ως προς το camera frame
        pose_cam = PoseStamped()
        pose_cam.header = self.pointcloud_msg.header
        pose_cam.pose.position.x = x
        pose_cam.pose.position.y = y
        pose_cam.pose.position.z = z
        pose_cam.pose.orientation.w = 1.0  # identity quaternion

        # ΤF Μετασχηματισμός ως προς  base_link
        try:
            pose_base = self.tf_buffer.transform(
                pose_cam,
                "base_link",
                timeout=Duration(seconds=0.5),
            )
        except Exception as e:
            self.get_logger().warn(f"TF transform failed: {e}")
            return
        
        # Δημοσιεύει το αποτέλεσμα ως pose_base στο topic /preinsert_pose
        self.pub_pose.publish(pose_base)

        self.good_count += 1
        self.get_logger().info(
            f"Inlet (base_link) medianROI: "
            f"x={pose_base.pose.position.x:.3f}, "
            f"y={pose_base.pose.position.y:.3f}, "
            f"z={pose_base.pose.position.z:.3f} "
            f"(good {self.good_count}/{self.lock_after_good})"
        )

        if self.good_count >= self.lock_after_good:
            self.done = True
            self.get_logger().info("Published inlet pose. Shutting down.")
            rclpy.shutdown()


def main():
    rclpy.init()
    node = Inlet3DFromPointcloud()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
