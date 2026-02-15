#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class InletImageSaverClick(Node):
    def __init__(self):
        super().__init__("inlet_image_saver_click")

        self.declare_parameter("image_topic", "/camera/image")
        self.declare_parameter("save_dir", "inlet_dataset")
        self.declare_parameter("max_images", 200)
        self.declare_parameter("window_name", "Wrist Camera (press 's' to save, 'q' to quit)")

        self.image_topic = self.get_parameter("image_topic").value
        self.save_dir = self.get_parameter("save_dir").value
        self.max_images = int(self.get_parameter("max_images").value)
        self.window_name = self.get_parameter("window_name").value

        os.makedirs(self.save_dir, exist_ok=True)

        self.bridge = CvBridge()
        self.img_count = 0
        self.latest_cv_image = None

        self.sub = self.create_subscription(Image, self.image_topic, self.image_callback, 10)

        # Timer for UI loop (OpenCV waitKey must run in main thread-ish)
        self.timer = self.create_timer(0.03, self.ui_loop)  # ~33 Hz

        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

        self.get_logger().info(
            f"Click-to-save image saver started.\n"
            f"Topic: {self.image_topic}\n"
            f"Directory: {self.save_dir}\n"
            f"Keys: 's' save, 'q' quit"
        )

    def image_callback(self, msg: Image):
        try:
            self.latest_cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge error: {e}")
            self.latest_cv_image = None

    def ui_loop(self):
        if self.latest_cv_image is None:
            return

        cv2.imshow(self.window_name, self.latest_cv_image)
        key = cv2.waitKey(1) & 0xFF

        # Quit
        if key in (ord('q'), 27):  # 'q' or ESC
            self.get_logger().info("Quitting...")
            rclpy.shutdown()
            return

        # Save
        if key == ord('s'):
            if self.img_count >= self.max_images:
                self.get_logger().warn("Max images reached, not saving.")
                return

            filename = os.path.join(self.save_dir, f"inlet_{self.img_count:04d}.png")
            ok = cv2.imwrite(filename, self.latest_cv_image)
            if ok:
                self.get_logger().info(f"Saved image: {filename}")
                self.img_count += 1
            else:
                self.get_logger().warn(f"Failed to save image: {filename}")

    def destroy_node(self):
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = InletImageSaverClick()
    rclpy.spin(node)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
