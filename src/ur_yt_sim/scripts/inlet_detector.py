#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np
import torch.serialization as ts


from ultralytics.nn.modules.conv import Conv, Concat
from ultralytics.nn.modules.block import C2f, SPPF, Bottleneck, DFL
from ultralytics.nn.modules.head import Detect
from ultralytics.utils.loss import v8DetectionLoss, BboxLoss
from ultralytics.utils.tal import TaskAlignedAssigner
from torch.nn import ModuleList, Conv2d, BatchNorm2d, Upsample, MaxPool2d
from torch.nn.modules.activation import SiLU
from ultralytics.utils import IterableSimpleNamespace
from ultralytics.nn.tasks import DetectionModel
from torch.nn import Sequential
from torch.nn import BCEWithLogitsLoss
from geometry_msgs.msg import PointStamped



ts.add_safe_globals([
    Conv, Concat, C2f, SPPF, Bottleneck, DFL, Detect,
    v8DetectionLoss, BboxLoss, TaskAlignedAssigner,
    ModuleList, Conv2d, BatchNorm2d, Upsample, MaxPool2d,
    SiLU, IterableSimpleNamespace, DetectionModel, Sequential, 
    BCEWithLogitsLoss,
    

])
   
class InletDetector2D(Node):

    def __init__(self):
        super().__init__("inlet_detector_2d")

        self.bridge = CvBridge()
        #subscribe to RGB image
        self.sub = self.create_subscription(
            Image, "/camera/image", self.image_cb, 10
        )

        self.pub = self.create_publisher(
            PointStamped, "/inlet/center_px", 10
        )
        
        self.get_logger().info("Loading YOLO model...")
        self.model = YOLO("/root/ros2_ws/src/ur_yt_sim/models/runs/detect/train3/weights/best.pt")

        self.get_logger().info("InletDetector2D ready (YOLO Ultralytics)")
        from collections import deque

        self.centers = deque(maxlen=8)
        self.areas = deque(maxlen=8)
        self.ratios = deque(maxlen=8)

        self.filtered_center = None
    def image_cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")  

        results = self.model(frame, conf=0.8)[0]
        if len(results.boxes) == 0:
            return

        boxes = results.boxes

        # Επιλογή του καλύτερου detection σε confidence x bbox area
        scores = []
        for b in boxes:
            x1, y1, x2, y2 = b.xyxy.cpu().numpy()[0]
            area = (x2 - x1) * (y2 - y1)
            scores.append(float(b.conf) * area)

        box = boxes[int(np.argmax(scores))]
        conf = float(box.conf)

        #Υπολογισμούς του κεντρου του bbox
        x1, y1, x2, y2 = box.xyxy.cpu().numpy()[0]
        cx = float(np.clip((x1 + x2) / 2, 0.0, 639.0))
        cy = float(np.clip((y1 + y2) / 2, 0.0, 479.0))

        w = x2 - x1
        h = y2 - y1
        area = w * h
        ratio = w / max(h, 1.0)

        self.centers.append((cx, cy))
        self.areas.append(area)
        self.ratios.append(ratio)

        if len(self.centers) < 5:
            return

        area_std = np.std(self.areas) / max(np.mean(self.areas), 1e-6)
        ratio_std = np.std(self.ratios) / max(np.mean(self.ratios), 1e-6)

        if area_std > 0.35 or ratio_std > 0.25:
            return

        alpha = 0.3
        if self.filtered_center is None:
            self.filtered_center = np.array([cx, cy], dtype=np.float32)
        else:
            self.filtered_center = alpha * np.array([cx, cy], dtype=np.float32) + (1 - alpha) * self.filtered_center

        msg_out = PointStamped()
        msg_out.header = msg.header
        msg_out.point.x = float(self.filtered_center[0])
        msg_out.point.y = float(self.filtered_center[1])
        msg_out.point.z = float(np.sqrt(area))
        self.pub.publish(msg_out)

        self.get_logger().info(
            f"Inlet stable @ ({msg_out.point.x:.1f}, {msg_out.point.y:.1f}) conf={conf:.2f}"
        )   

        
def main():
    rclpy.init()
    rclpy.spin(InletDetector2D())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
