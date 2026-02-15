#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

import numpy as np
import cv2

from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PointStamped, PoseStamped
import sensor_msgs_py.point_cloud2 as pc2

import tf2_ros
import tf2_geometry_msgs  # IMPORTANT

from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer
from sensor_msgs.msg import Image as ImageMsg

def quat_from_R(R: np.ndarray):
    """Convert 3x3 rotation matrix to quaternion (x,y,z,w)."""
    # Robust conversion
    t = np.trace(R)
    if t > 0.0:
        S = np.sqrt(t + 1.0) * 2.0
        qw = 0.25 * S
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S
    else:
        if (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
            S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
            qw = (R[2, 1] - R[1, 2]) / S
            qx = 0.25 * S
            qy = (R[0, 1] + R[1, 0]) / S
            qz = (R[0, 2] + R[2, 0]) / S
        elif R[1, 1] > R[2, 2]:
            S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
            qw = (R[0, 2] - R[2, 0]) / S
            qx = (R[0, 1] + R[1, 0]) / S
            qy = 0.25 * S
            qz = (R[1, 2] + R[2, 1]) / S
        else:
            S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
            qw = (R[1, 0] - R[0, 1]) / S
            qx = (R[0, 2] + R[2, 0]) / S
            qy = (R[1, 2] + R[2, 1]) / S
            qz = 0.25 * S

    q = np.array([qx, qy, qz, qw], dtype=np.float64)
    # normalize
    n = np.linalg.norm(q)
    if n < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    q /= n
    return float(q[0]), float(q[1]), float(q[2]), float(q[3])


class InletFinalPoseFromCloud(Node):
    def __init__(self):
        super().__init__("inlet_final_pose_from_cloud")

        # Params
        self.target_frame = self.declare_parameter("target_frame", "base_link").value
        self.final_topic = self.declare_parameter("final_topic", "/inlet/final_pose").value

        # Geometry from your CAD
        self.dc_distance_m = float(self.declare_parameter("dc_distance_m", 0.027).value)
        self.center7_up_m = float(self.declare_parameter("center7_up_m", 0.0495).value)

        # Crop / circle detection tuning
        self.crop_half = int(self.declare_parameter("crop_half_px", 140).value)  # crop = (2*half)x(2*half)
        self.roi_half = int(self.declare_parameter("roi_half_px", 4).value)      # median ROI in pointcloud
        self.min_roi_points = int(self.declare_parameter("min_roi_points", 20).value)

        self.hough_dp = float(self.declare_parameter("hough_dp", 1.2).value)
        self.hough_minDist = float(self.declare_parameter("hough_minDist", 35.0).value)
        self.hough_param1 = float(self.declare_parameter("hough_param1", 120.0).value)
        self.hough_param2 = float(self.declare_parameter("hough_param2", 22.0).value)
        self.hough_minR = int(self.declare_parameter("hough_minRadius", 10).value)
        self.hough_maxR = int(self.declare_parameter("hough_maxRadius", 45).value)

        # Plane fit tuning
        self.plane_keep_percentile = float(self.declare_parameter("plane_keep_percentile", 35.0).value)
        self.plane_min_points = int(self.declare_parameter("plane_min_points", 300).value)

        # ROS pubs
        self.pub_final = self.create_publisher(PoseStamped, self.final_topic, 10)

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        #Debug
        self.pub_dbg = self.create_publisher(ImageMsg, "/inlet/debug_dc_crop", 1)
        self.debug = bool(self.declare_parameter("debug", True).value)

        # Sync subs: image + pointcloud + bbox center
        self.bridge = CvBridge()
        self.sub_img = Subscriber(self, Image, "/camera/image")
        self.sub_pc = Subscriber(self, PointCloud2, "/camera/points")
        self.sub_px = Subscriber(self, PointStamped, "/inlet/center_px")

        self.sync = ApproximateTimeSynchronizer(
            [self.sub_img, self.sub_pc, self.sub_px],
            queue_size=10,
            slop=0.12,
            allow_headerless=False,
        )
        self.sync.registerCallback(self.synced_cb)

        self.get_logger().info("Inlet final-pose estimator started (image+cloud+bbox synced).")

    def median_3d_at(self, cloud_hw3: np.ndarray, u: int, v: int):
        H, W, _ = cloud_hw3.shape
        u0 = max(0, u - self.roi_half)
        u1 = min(W - 1, u + self.roi_half)
        v0 = max(0, v - self.roi_half)
        v1 = min(H - 1, v + self.roi_half)

        region = cloud_hw3[v0:v1 + 1, u0:u1 + 1, :].reshape(-1, 3)
        region = region[np.isfinite(region).all(axis=1)]

        if region.shape[0] < self.min_roi_points:
            return None

        xyz = np.median(region, axis=0)
        if not np.isfinite(xyz).all():
            return None
        return xyz.astype(np.float64)

    def fit_plane_normal(self, pts: np.ndarray, center_hint: np.ndarray):
        """
        PCA plane fit: return unit normal.
        Uses a depth-based filter to keep points likely belonging to the panel plane.
        """
        if pts.shape[0] < self.plane_min_points:
            return None

        # Keep points near the "front" surface (closest to camera) using z percentile
        z = pts[:, 2]
        z_thr = np.percentile(z, self.plane_keep_percentile)
        inliers = pts[z <= z_thr]
        if inliers.shape[0] < self.plane_min_points:
            inliers = pts  # fallback

        mu = np.mean(inliers, axis=0)
        X = inliers - mu

        # SVD: smallest singular vector is normal
        _, _, vh = np.linalg.svd(X, full_matrices=False)
        n = vh[-1, :]
        n_norm = np.linalg.norm(n)
        if n_norm < 1e-9:
            return None
        n = n / n_norm

        # Make normal point toward the camera (camera at origin in camera frame)
        # If dot(n, (camera - point)) < 0 => flip
        to_cam = -center_hint
        if np.dot(n, to_cam) < 0.0:
            n = -n
        return n

    def synced_cb(self, img_msg: Image, pc_msg: PointCloud2, px_msg: PointStamped):
        # bbox center (pixels)
        u_c = int(px_msg.point.x)
        v_c = int(px_msg.point.y)

        H = int(pc_msg.height)
        W = int(pc_msg.width)

        if H <= 1 or W <= 1:
            self.get_logger().warn(f"Pointcloud not organized (H={H}, W={W}).")
            return

        if not (0 <= u_c < W and 0 <= v_c < H):
            self.get_logger().warn(f"Center pixel out of bounds: u={u_c}, v={v_c}")
            return

        # Convert pointcloud -> (H,W,3)
        try:
            cloud = pc2.read_points_numpy(pc_msg, field_names=("x", "y", "z"))
            cloud = cloud.reshape(H, W, 3)
        except Exception as e:
            self.get_logger().warn(f"PointCloud read/reshape error: {e}")
            return

        # Convert image
        try:
            cv_img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge error: {e}")
            return

        # Crop around center
        x0 = max(0, u_c - self.crop_half)
        x1 = min(W - 1, u_c + self.crop_half)
        y0 = max(0, v_c - self.crop_half)
        y1 = min(H - 1, v_c + self.crop_half)

        crop = cv_img[y0:y1 + 1, x0:x1 + 1]
        if crop.size == 0:
            return

        gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (7, 7), 1.5)

        # Heuristic: DC pins are in the lower half of the inlet
        h, w = gray.shape[:2]
        lower_y0 = int(0.45 * h)
        gray_eq = cv2.equalizeHist(gray)
        lower = gray_eq[lower_y0:h, :]

        circles = cv2.HoughCircles(
            lower,
            cv2.HOUGH_GRADIENT,
            dp=self.hough_dp,
            minDist=self.hough_minDist,
            param1=self.hough_param1,
            param2=self.hough_param2,
            minRadius=self.hough_minR,
            maxRadius=self.hough_maxR,
        )
        # --- DEBUG VISUALIZATION ---
        if self.debug:
            dbg = crop.copy()

            # draw crop bounds reference line where "lower" starts
            cv2.line(dbg, (0, lower_y0), (dbg.shape[1]-1, lower_y0), (0, 255, 255), 2)

            # if circles found, draw them
            if circles is not None:
                for (cx, cy, r) in np.round(circles[0, :]).astype(np.int32):
                    # circles are in LOWER coords, map to crop coords
                    cy_full = cy + lower_y0
                    cv2.circle(dbg, (cx, cy_full), r, (0, 255, 0), 2)
                    cv2.circle(dbg, (cx, cy_full), 2, (0, 0, 255), 3)

            # publish debug image
            dbg_msg = self.bridge.cv2_to_imgmsg(dbg, encoding="bgr8")
            dbg_msg.header = img_msg.header
            self.pub_dbg.publish(dbg_msg)
        # --- END DEBUG ---
        if circles is None:
            self.get_logger().warn("No circles found (DC pins). Tune Hough params / crop.")
            return

        circles = np.round(circles[0, :]).astype(np.int32)  # (x,y,r) in LOWER coords
        # pick two largest circles
        circles = circles[np.argsort(-circles[:, 2])]
        circles = circles[: min(4, circles.shape[0])]  # keep top few by radius

        if circles.shape[0] < 2:
            self.get_logger().warn("Less than 2 circles found.")
            return

        # Choose best pair: maximize horizontal separation
        best_pair = None
        best_sep = -1
        for i in range(circles.shape[0]):
            for j in range(i + 1, circles.shape[0]):
                sep = abs(circles[i, 0] - circles[j, 0])
                if sep > best_sep:
                    best_sep = sep
                    best_pair = (circles[i], circles[j])

        c1, c2 = best_pair

        # Convert lower-crop coords -> full image coords
        # lower starts at y_offset within crop
        lower_y0 = int(0.45 * h)

        u1 = x0 + int(c1[0])
        v1 = y0 + lower_y0 + int(c1[1])

        u2 = x0 + int(c2[0])
        v2 = y0 + lower_y0 + int(c2[1])

        # Order left/right by u
        if u2 < u1:
            (u1, v1, u2, v2) = (u2, v2, u1, v1)

        P1 = self.median_3d_at(cloud, u1, v1)
        P2 = self.median_3d_at(cloud, u2, v2)

        if P1 is None or P2 is None:
            self.get_logger().warn("Failed to lift one/both DC centers to 3D (insufficient ROI points).")
            return

        dc_dist = float(np.linalg.norm(P2 - P1))
        # simple consistency check
        if abs(dc_dist - self.dc_distance_m) > 0.02:  # 2 cm tolerance
            self.get_logger().warn(f"DC distance check: got {dc_dist:.3f}m, expected ~{self.dc_distance_m:.3f}m")

        M = 0.5 * (P1 + P2)
        x_hat = (P2 - P1)
        nx = np.linalg.norm(x_hat)
        if nx < 1e-9:
            self.get_logger().warn("Degenerate DC axis.")
            return
        x_hat = x_hat / nx

        # Plane fit points: take a ring-ish region around crop in the pointcloud
        # Use the same crop bounds on cloud, but keep only finite points
        cloud_crop = cloud[y0:y1 + 1, x0:x1 + 1, :].reshape(-1, 3)
        cloud_crop = cloud_crop[np.isfinite(cloud_crop).all(axis=1)]
        if cloud_crop.shape[0] < self.plane_min_points:
            self.get_logger().warn(f"Not enough points for plane fit: {cloud_crop.shape[0]}")
            return

        z_hat = self.fit_plane_normal(cloud_crop, center_hint=M)
        if z_hat is None:
            self.get_logger().warn("Plane fit failed.")
            return

        # y = z x x
        y_hat = np.cross(z_hat, x_hat)
        ny = np.linalg.norm(y_hat)
        if ny < 1e-9:
            self.get_logger().warn("Degenerate y axis (z parallel x).")
            return
        y_hat = y_hat / ny

        # Re-orthogonalize x to be safe: x = y x z
        x_hat = np.cross(y_hat, z_hat)
        x_hat /= np.linalg.norm(x_hat)

        # Target: central of 7 pins is 49.5mm "up" from midpoint of DCs (local +y)
        P_center7 = M + self.center7_up_m * y_hat

        # Build rotation matrix (columns = axes in camera frame)
        R = np.column_stack((x_hat, y_hat, z_hat))
        qx, qy, qz, qw = quat_from_R(R)

        # Pose in camera frame
        pose_cam = PoseStamped()
        pose_cam.header = pc_msg.header  # frame_id should be camera_optical_link
        pose_cam.pose.position.x = float(P_center7[0])
        pose_cam.pose.position.y = float(P_center7[1])
        pose_cam.pose.position.z = float(P_center7[2])
        pose_cam.pose.orientation.x = qx
        pose_cam.pose.orientation.y = qy
        pose_cam.pose.orientation.z = qz
        pose_cam.pose.orientation.w = qw

        # Transform to base_link (or chosen target frame)
        try:
            pose_out = self.tf_buffer.transform(
                pose_cam,
                self.target_frame,
                timeout=Duration(seconds=0.5),
            )
        except Exception as e:
            self.get_logger().warn(f"TF transform failed: {e}")
            return

        self.pub_final.publish(pose_out)

        self.get_logger().info(
            f"FINAL pose ({self.target_frame}): "
            f"p=[{pose_out.pose.position.x:.3f}, {pose_out.pose.position.y:.3f}, {pose_out.pose.position.z:.3f}] "
            f"DCdist={dc_dist:.3f}m"
        )


def main():
    rclpy.init()
    node = InletFinalPoseFromCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
