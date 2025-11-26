"""
ROS2 node that:
- Opens an Intel RealSense D435 camera using pyrealsense2 over USB
- Publishes:
    - color frames as sensor_msgs/msg/Image on /realsense/color/image_raw
    - infrared (stereo-ish) frames on /realsense/stereo/image_raw
"""

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import pyrealsense2 as rs
import numpy as np


class RealSensePublisher(Node):
    def __init__(self):
        super().__init__('realsense_camera_publisher')

        # Parameters (can be overridden via ROS2 params)
        self.color_topic = self.declare_parameter(
            'color_topic', '/realsense/color/image_raw'
        ).get_parameter_value().string_value

        self.stereo_topic = self.declare_parameter(
            'stereo_topic', '/realsense/stereo/image_raw'
        ).get_parameter_value().string_value

        self.get_logger().info(f"Publishing RealSense color frames on:  {self.color_topic}")
        self.get_logger().info(f"Publishing RealSense stereo frames on: {self.stereo_topic}")

        # Publishers
        self.color_pub = self.create_publisher(Image, self.color_topic, 10)
        self.stereo_pub = self.create_publisher(Image, self.stereo_topic, 10)
        self.bridge = CvBridge()

        # Setup RealSense pipeline
        self.pipeline = rs.pipeline()
        config = rs.config()

        # Enable color stream
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Enable infrared stream (acts as a "stereo" view)
        # For D435: infrared 1 = left, 2 = right. We’ll use 1 here.
        config.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 30)

        self.get_logger().info("Starting RealSense pipeline...")
        self.pipeline.start(config)
        self.get_logger().info("RealSense pipeline started.")

        # Timer for publishing frames
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)

    def timer_callback(self):
        # Non-blocking poll to avoid blocking the executor
        frames = self.pipeline.poll_for_frames()
        if not frames:
            return

        color_frame = frames.get_color_frame()
        ir_frame = frames.get_infrared_frame(1)  # infrared camera 1

        if not color_frame or not ir_frame:
            return

        # Convert RealSense frames to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())   # BGR
        infrared_image = np.asanyarray(ir_frame.get_data())   # mono8

        # Get a timestamp for both messages
        stamp = self.get_clock().now().to_msg()

        # Publish color
        color_msg = self.bridge.cv2_to_imgmsg(color_image, encoding='bgr8')
        color_msg.header.stamp = stamp
        color_msg.header.frame_id = 'realsense_color_optical_frame'
        self.color_pub.publish(color_msg)

        # Publish infrared as stereo (mono8)
        stereo_msg = self.bridge.cv2_to_imgmsg(infrared_image, encoding='mono8')
        stereo_msg.header.stamp = stamp
        stereo_msg.header.frame_id = 'realsense_infra1_optical_frame'
        self.stereo_pub.publish(stereo_msg)
        print(f"Publishing frame {stamp}")
	
    def destroy_node(self):
        # Stop the RealSense pipeline cleanly
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

