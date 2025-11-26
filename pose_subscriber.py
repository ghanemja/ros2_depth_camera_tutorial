#!/usr/bin/env python3
"""
ROS2 node that:
- Subscribes to /realsense/color/image_raw
- Runs MediaPipe Pose on CPU
- Draws a skeleton on each frame and shows it via OpenCV imshow
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import mediapipe as mp


class PoseVisualizer(Node):
    def __init__(self):
        super().__init__('pose_visualizer')

        # Subscribe to the RGB topic from the RealSense publisher
        self.subscription = self.create_subscription(
            Image,
            '/realsense/color/image_raw',
            self.image_callback,
            10
        )

        self.bridge = CvBridge()

        # Set up MediaPipe Pose (CPU-only is fine)
        self.mp_pose = mp.solutions.pose
        self.mp_drawing = mp.solutions.drawing_utils
        self.pose = self.mp_pose.Pose(
            static_image_mode=False,
            model_complexity=1,     # 0, 1, or 2 (higher = heavier / more accurate)
            enable_segmentation=False,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5,
        )

        self.get_logger().info("PoseVisualizer initialized and subscribed to /realsense/color/image_raw")

    def image_callback(self, msg: Image):
        # Convert ROS Image -> OpenCV BGR
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # MediaPipe expects RGB
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Run pose estimation
        results = self.pose.process(rgb)

        # Draw pose landmarks on the original BGR frame
        if results.pose_landmarks:
            self.mp_drawing.draw_landmarks(
                frame,
                results.pose_landmarks,
                self.mp_pose.POSE_CONNECTIONS
            )

        # Show the frame in an OpenCV window
        cv2.imshow("RealSense Pose Estimation", frame)
        # Small wait so the window updates; also lets user close it with 'q'
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.get_logger().info("Received 'q' keypress, shutting down node...")
            # rclpy.shutdown() will be called from main
            raise KeyboardInterrupt

    def destroy_node(self):
        # Clean up any resources
        self.get_logger().info("Destroying PoseVisualizer node...")
        self.pose.close()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PoseVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("PoseVisualizer interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

