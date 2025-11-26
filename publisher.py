"""
ROS2 node that:
- Opens an Intel RealSense D435 camera using pyrealsense2 over USB
- Publishes:
    - color frames as sensor_msgs/msg/Image on /realsense/color/image_raw
    - infrared (stereo-ish) frames on /realsense/stereo/image_raw
    - colorized depth frames on /realsense/depth_vis (for visualization)
"""

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import pyrealsense2 as rs
import numpy as np
import cv2


class RealSensePublisher(Node):
    def __init__(self):
        super().__init__('realsense_camera_publisher')

        self.color_topic = self.declare_parameter(
            'color_topic', '/realsense/color/image_raw'
        ).get_parameter_value().string_value

        self.stereo_topic = self.declare_parameter(
            'stereo_topic', '/realsense/stereo/image_raw'
        ).get_parameter_value().string_value

        self.depth_vis_topic = self.declare_parameter(
            'depth_vis_topic', '/realsense/depth_vis'
        ).get_parameter_value().string_value

        self.get_logger().info(f"Publishing RealSense color frames on:  {self.color_topic}")
        self.get_logger().info(f"Publishing RealSense stereo frames on: {self.stereo_topic}")
        self.get_logger().info(f"Publishing colorized depth on:         {self.depth_vis_topic}")

        self.color_pub = self.create_publisher(Image, self.color_topic, 10)
        self.stereo_pub = self.create_publisher(Image, self.stereo_topic, 10)
        self.depth_vis_pub = self.create_publisher(Image, self.depth_vis_topic, 10)

        self.bridge = CvBridge()

        self.pipeline = rs.pipeline()
        config = rs.config()

        # Enable color stream (BGR)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Enable infrared stream (acts as "stereo-ish" view)
        # For D435: infrared 1 = left, 2 = right. We’ll use 1 here.
        config.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 30)

        # Enable depth stream (z16 = uint16 depth in mm)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        self.get_logger().info("Starting RealSense pipeline...")
        self.pipeline.start(config)
        self.get_logger().info("RealSense pipeline started.")

        # Timer for publishing frames (30 Hz)
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)

    def timer_callback(self):
        frames = self.pipeline.poll_for_frames()
        if not frames:
            return

        color_frame = frames.get_color_frame()
        ir_frame = frames.get_infrared_frame(1)
        depth_frame = frames.get_depth_frame()

        if not color_frame or not ir_frame or not depth_frame:
            return

        # Convert RealSense frames to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())       # BGR
        infrared_image = np.asanyarray(ir_frame.get_data())       # mono8
        depth_image = np.asanyarray(depth_frame.get_data())       # uint16 depth (mm)

        stamp = self.get_clock().now().to_msg()

	# this is where we publish!!
        color_msg = self.bridge.cv2_to_imgmsg(color_image, encoding='bgr8')
        color_msg.header.stamp = stamp
        color_msg.header.frame_id = 'realsense_color_optical_frame'
        self.color_pub.publish(color_msg)

        stereo_msg = self.bridge.cv2_to_imgmsg(infrared_image, encoding='mono8')
        stereo_msg.header.stamp = stamp
        stereo_msg.header.frame_id = 'realsense_infra1_optical_frame'
        self.stereo_pub.publish(stereo_msg)

	# STUDENTS; ADD HERE FOR DEPTH TUTORIAL
        # Normalize depth to [0, 255] for visualization
        depth_norm = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
        depth_norm = depth_norm.astype(np.uint8)
        depth_vis_color = cv2.applyColorMap(depth_norm, cv2.COLORMAP_TURBO)

        depth_msg = self.bridge.cv2_to_imgmsg(depth_vis_color, encoding='bgr8')
        depth_msg.header.stamp = stamp
        depth_msg.header.frame_id = 'realsense_depth_optical_frame'
        self.depth_vis_pub.publish(depth_msg)

        # comment and uncomment this as needed for debugging
        # self.get_logger().info(f"Published frame at {stamp.sec}.{stamp.nanosec}")

    def destroy_node(self):
        try:
            self.get_logger().info("Stopping RealSense pipeline...")
            self.pipeline.stop()
        except Exception as e:
            self.get_logger().warn(f"Error stopping pipeline: {e}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RealSensePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down RealSense publisher...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

