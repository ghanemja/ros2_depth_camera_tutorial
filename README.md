# ROS2 RealSense D435 HTTP Streaming Demo

## Overview (What You Will Be Creating)

The objective of this demo is to run your own ROS2 Python nodes on a desktop/laptop with an Intel RealSense D435 depth camera connected over USB. You will:

* Publish RealSense **color RGB frames** into ROS2 from a Python node.
* Publish RealSense **infrared (IR) frames** from the left IR sensor as a “stereo-ish” stream into ROS2.
* Subscribe to both topics from a second node.
* Stream the live RGB and IR feeds over HTTP as MJPEG streams with a browser toggle between modes.

<video src="static/result.webm" controls loop muted playsinline style="max-width: 100%; border-radius: 12px;"></video>

The RGB stream is a standard color webcam-like feed. The IR “stereo” stream is the raw mono8 infrared image from the D435’s left IR sensor, which is what the camera uses internally for depth estimation (it is **not** a depth map).

---

## PREREQUISITES

You should have:

* An Intel RealSense D435 plugged into your computer with the USB cable
  (must be a USB 3.0 port – this is the USB port directly next to the power).
* A working ROS2 desktop install (ros2 and colcon) on Linux.
* Python 3 available in your ROS2 environment.

### Install the RealSense SDK + Python libraries

Follow Intel's documentation to install `librealsense` (RealSense SDK) via the following commands:

```bash
sudo apt-get update
sudo apt-get install apt-transport-https curl gnupg2 software-properties-common -y

sudo mkdir -p /etc/apt/keyrings
curl -fsSL https://librealsense.intel.com/Debian/librealsense.pgp \
  | sudo tee /etc/apt/keyrings/librealsense.gpg > /dev/null

echo "deb [signed-by=/etc/apt/keyrings/librealsense.gpg] https://librealsense.intel.com/Debian/apt-repo focal main" \
  | sudo tee /etc/apt/sources.list.d/librealsense.list

sudo apt-get update

cd /tmp
wget http://archive.ubuntu.com/ubuntu/pool/main/o/openssl/libssl1.1_1.1.1f-1ubuntu2.24_amd64.deb
sudo dpkg -i libssl1.1_1.1.1f-1ubuntu2.24_amd64.deb

sudo apt-get install librealsense2-utils librealsense2-dev librealsense2-dbg
```

Install the Python dependencies in your ROS2 environment:

```bash
sudo apt-get install ros-humble-cv-bridge
pip install opencv-python "numpy<2.0"
```

### Sanity check: test the USB camera connection

Before using ROS2, make sure the camera works over USB with the RealSense viewer:

```bash
realsense-viewer
```

Once launched, you should see the camera in the RealSense Viewer and be able to view the streams.

You should see the camera details pop up on the left side of the viewer with the text **“Add Source (1 available)”**. Expand the **Stereo Module** and **RGB Camera** carets to play more with the display and post-processing features.

---

## CONTENTS

* SECTION A: Environment Setup
* SECTION B: Running the RealSense Pub/Sub + HTTP Demo
* SECTION C: Networking across multiple machines
* Explanations

---

## SECTION A: Environment Setup

### Step 1: Source ROS2

Open a terminal and source your ROS2 environment:

```bash
source /opt/ros/humble/setup.bash
```

### Step 2: Create / use a workspace folder

`cd` into the directory which holds this tutorial (which will also contain `publisher.py`, `subscriber.py`, and optionally `index.html`):

```bash
cd /path/to/ros2_depth_camera_tutorial
```

---

## SECTION B: RealSense Pub/Sub + HTTP Streaming

### Step 1: RealSense Publisher Node (Python file #1)

The streams will be published in two topics:

* `/realsense/color/image_raw` → `rgb.mjpg`

  * Unprocessed, full-color BGR frame directly from the D435’s RGB sensor.
  * 640×480, 30 FPS, no depth, IR, or filtering.
  * This is just the camera’s raw color output published as JPEG frames (webcam-style).

* `/realsense/stereo/image_raw` → `stereo.mjpg`

  * Infrared stream from the RealSense.
  * Monochrome (`mono8`) direct IR sensor photon values.
  * For display, it is converted to 3-channel.
  * This is what is used for depth estimation before any processing occurs.
  * In later steps, you can apply math to this imagery data to get depth and perform other vision operations.

In `/path/to/ros2_depth_camera_tutorial`, create a file called `publisher.py` with the following content:

```python
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

        self.get_logger().info(
            f"Publishing RealSense color frames on:  {self.color_topic}"
        )
        self.get_logger().info(
            f"Publishing RealSense stereo frames on: {self.stereo_topic}"
        )

        # Publishers
        self.color_pub = self.create_publisher(Image, self.color_topic, 10)
        self.stereo_pub = self.create_publisher(Image, self.stereo_topic, 10)
        self.bridge = CvBridge()

        # Setup RealSense pipeline
        self.pipeline = rs.pipeline()
        config = rs.config()

        # Enable color stream (RGB)
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
        ir_frame = frames.get_infrared_frame(1)  # infrared camera 1 (left IR)

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
```

The RGB stream (`/realsense/color/image_raw`) is a standard 640×480, 30 FPS, `bgr8` color image.
The stereo stream (`/realsense/stereo/image_raw`) is a raw `mono8` infrared image from the left IR sensor. It is what the D435 uses internally for depth calculation, not a depth map itself.

---

### Step 2: HTTP Streaming Subscriber Node (Python file #2)

In the same folder, create a file called `subscriber.py` with the following content.
This node subscribes to both the RGB and IR topics and exposes them as MJPEG streams over HTTP, plus a simple HTML page with an RGB / Stereo toggle.

```python
"""
ROS2 node that:
- Subscribes to:
    /realsense/color/image_raw
    /realsense/stereo/image_raw
- Streams both as MJPEG over HTTP:
    /rgb.mjpg
    /stereo.mjpg
- Serves an HTML page at http://<this-machine-ip>:8000/
  with a toggle to switch between RGB and Stereo.
"""

import threading
import time
import sys
from typing import Optional

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import urlparse

import cv2

# Global frame storage
_latest_rgb_jpeg_lock = threading.Lock()
_latest_rgb_jpeg: Optional[bytes] = None

_latest_stereo_jpeg_lock = threading.Lock()
_latest_stereo_jpeg: Optional[bytes] = None


class RealSenseHttpSubscriber(Node):
    def __init__(self):
        super().__init__('realsense_http_subscriber')

        # Fixed topics to match the publisher
        self.color_topic = "/realsense/color/image_raw"
        self.stereo_topic = "/realsense/stereo/image_raw"

        self.get_logger().info(f"Subscribing to RGB:    {self.color_topic}")
        self.get_logger().info(f"Subscribing to Stereo: {self.stereo_topic}")

        self.bridge = CvBridge()

        # Subscribers
        self.rgb_sub = self.create_subscription(
            Image, self.color_topic, self.rgb_callback, 10
        )
        self.stereo_sub = self.create_subscription(
            Image, self.stereo_topic, self.stereo_callback, 10
        )

        # HTTP server
        self.get_logger().info("Starting HTTP server on port 8000...")
        self.http_server = start_http_server(port=8000)
        self.get_logger().info("HTTP server started.")

    # ---------------------------------------------------------
    # Callbacks
    # ---------------------------------------------------------
    def rgb_callback(self, msg: Image):
        global _latest_rgb_jpeg

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warning(f"[RGB] Conversion error: {e}")
            return

        ok, jpeg = cv2.imencode('.jpg', frame)
        if not ok:
            self.get_logger().warning("[RGB] JPEG encoding failed")
            return

        with _latest_rgb_jpeg_lock:
            _latest_rgb_jpeg = jpeg.tobytes()

    def stereo_callback(self, msg: Image):
        global _latest_stereo_jpeg

        try:
            # Publisher sends mono8 (infrared)
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            # Convert mono to 3-channel for consistent JPEG display
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        except Exception as e:
            self.get_logger().warning(f"[STEREO] Conversion error: {e}")
            return

        ok, jpeg = cv2.imencode('.jpg', img)
        if not ok:
            self.get_logger().warning("[STEREO] JPEG encoding failed")
            return

        with _latest_stereo_jpeg_lock:
            _latest_stereo_jpeg = jpeg.tobytes()

    def destroy_node(self):
        try:
            self.get_logger().info("Shutting down HTTP server...")
            self.http_server.shutdown()
            self.http_server.server_close()
        except Exception as e:
            self.get_logger().warning(f"Error shutting down HTTP server: {e}")
        super().destroy_node()


# -------------------------------------------------------------
# HTTP SERVER
# -------------------------------------------------------------
class StreamRequestHandler(BaseHTTPRequestHandler):
    # Avoid noisy default logging
    def log_message(self, format, *args):
        sys.stderr.write("[HTTP] " + format % args + "\n")

    def do_GET(self):
        # Strip query string (e.g. /rgb.mjpg?t=12345 -> /rgb.mjpg)
        parsed = urlparse(self.path)
        path = parsed.path

        if path in ("/", "/index.html"):
            self._serve_html()
        elif path == "/rgb.mjpg":
            self._stream(kind="rgb")
        elif path == "/stereo.mjpg":
            self._stream(kind="stereo")
        else:
            self.send_error(404, "Not Found")

    def _serve_html(self):
        # Single <img> element; JS toggles between /rgb.mjpg and /stereo.mjpg
        html = b"""<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <title>RealSense Live Stream</title>
  <style>
    body {
      background: #020617;
      color: #e5e7eb;
      font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      text-align: center;
      margin: 0;
      padding: 2rem;
    }
    h1 { margin-bottom: 1rem; }
    .buttons { margin-bottom: 1rem; }
    button {
      margin: 0 0.5rem;
      padding: 0.4rem 1rem;
      border-radius: 999px;
      border: 1px solid #475569;
      background: #020617;
      color: #e5e7eb;
      cursor: pointer;
    }
    button.active {
      background: #38bdf8;
      color: #020617;
    }
    img {
      max-width: 90vw;
      max-height: 70vh;
      border-radius: 12px;
      background: black;
      display: block;
      margin: 0 auto;
    }
    #modeLabel {
      margin-top: 0.5rem;
      color: #9ca3af;
    }
  </style>
</head>
<body>
  <h1>RealSense D435 Live Stream</h1>

  <div class="buttons">
    <button id="rgbBtn" class="active">RGB</button>
    <button id="stereoBtn">Stereo</button>
  </div>

  <!-- Single stream element -->
  <img id="stream" src="" alt="Camera stream">
  <div id="modeLabel">Mode: RGB</div>

<script>
  const img = document.getElementById('stream');
  const rgbBtn = document.getElementById('rgbBtn');
  const stereoBtn = document.getElementById('stereoBtn');
  const modeLabel = document.getElementById('modeLabel');

  let currentMode = null;

  function startStream(mode) {
    const base = mode === 'rgb' ? '/rgb.mjpg' : '/stereo.mjpg';
    const url = base + '?t=' + Date.now(); // cache-buster

    // Close previous connection by clearing src first
    img.src = '';
    setTimeout(() => {
      img.src = url;
    }, 50);

    currentMode = mode;
    modeLabel.textContent = 'Mode: ' + (mode === 'rgb' ? 'RGB' : 'Stereo');

    if (mode === 'rgb') {
      rgbBtn.classList.add('active');
      stereoBtn.classList.remove('active');
    } else {
      stereoBtn.classList.add('active');
      rgbBtn.classList.remove('active');
    }
  }

  rgbBtn.addEventListener('click', () => {
    if (currentMode !== 'rgb') startStream('rgb');
  });

  stereoBtn.addEventListener('click', () => {
    if (currentMode !== 'stereo') startStream('stereo');
  });

  // Default to RGB on load
  startStream('rgb');
</script>
</body>
</html>"""

        self.send_response(200)
        self.send_header("Content-Type", "text/html")
        self.send_header("Content-Length", str(len(html)))
        self.end_headers()
        self.wfile.write(html)

    # ---- MJPEG Streaming ----
    def _stream(self, kind: str):
        global _latest_rgb_jpeg, _latest_stereo_jpeg

        self.send_response(200)
        self.send_header("Age", "0")
        self.send_header("Cache-Control", "no-cache, private")
        self.send_header("Pragma", "no-cache")
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()

        while True:
            try:
                if kind == "rgb":
                    with _latest_rgb_jpeg_lock:
                        frame = _latest_rgb_jpeg
                else:
                    with _latest_stereo_jpeg_lock:
                        frame = _latest_stereo_jpeg

                if frame is None:
                    time.sleep(0.05)
                    continue

                # One MJPEG part
                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(
                    b"Content-Length: " + str(len(frame)).encode() + b"\r\n\r\n"
                )
                self.wfile.write(frame)
                self.wfile.write(b"\r\n")

                time.sleep(0.03)

            except (BrokenPipeError, ConnectionResetError):
                break
            except Exception as e:
                sys.stderr.write(f"[HTTP] Streaming error (%s): %s\n" % (kind, e))
                break


def start_http_server(host: str = "0.0.0.0", port: int = 8000) -> HTTPServer:
    server = HTTPServer((host, port), StreamRequestHandler)

    def _serve():
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass

    thread = threading.Thread(target=_serve, daemon=True)
    thread.start()
    return server


def main(args=None):
    rclpy.init(args=args)
    node = RealSenseHttpSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

The HTTP node acts like a tiny MJPEG server embedded inside your ROS2 node. It always keeps only one active MJPEG connection from the GUI (RGB or Stereo) at a time, which avoids issues with multiple simultaneous MJPEG consumers.

---

### Step 3: Run the demo in two terminals

In **terminal 1** (publisher):

```bash
cd /path/to/ros2_depth_camera_tutorial
source /opt/ros/humble/setup.bash
python3 publisher.py
```

In **terminal 2** (HTTP subscriber):

```bash
cd /path/to/ros2_depth_camera_tutorial
source /opt/ros/humble/setup.bash
python3 subscriber.py
```

Double check the topics are being published by opening a new terminal:

```bash
source /opt/ros/humble/setup.bash
ros2 topic list | grep realsense
```

You should see:

```text
/realsense/color/image_raw
/realsense/stereo/image_raw
```

Optionally, confirm encodings:

```bash
ros2 topic echo /realsense/color/image_raw --once
ros2 topic echo /realsense/stereo/image_raw --once
```

The color topic should show `encoding: "bgr8"`.
The stereo topic should show `encoding: "mono8"`.

Then open your browser to:

```text
http://localhost:8000/
```

You should see a live RealSense stream with a toggle between RGB (color) and Stereo (IR).
If you want to access it from another device on the same network, replace `localhost` with your machine's IP.

---

## SECTION C: Networking across multiple machines

To network across multiple machines:

1. Ensure both computers are on `neet01` or the same Wi-Fi network.
2. Set the environment variables so that both publisher and subscribers share the same ROS2 domain ID.

On the **publisher** computer connected to the depth camera:

```bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
sudo ufw disable
```

On the **subscriber** machine:

```bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
sudo ufw disable
```

Test that the subscriber computer can see the topics via:

```bash
ros2 topic list
```

You should see:

```text
/realsense/color/image_raw
/realsense/stereo/image_raw
```

Get the IP address of the host machine via:

```bash
hostname -I
```

Now try accessing the server on the other computer by putting the IP address you just found into the browser URL:

```text
http://host_IP_address:8000/
```

You can also hit the raw MJPEG endpoints directly:

```text
http://host_IP_address:8000/rgb.mjpg
http://host_IP_address:8000/stereo.mjpg
```

---

## Explanations

There are 3 main layers in this setup – the host OS, the ROS2 layer, and the Python scripts.

### 1. Host OS + RealSense SDK

Your desktop OS (Ubuntu) runs the Intel RealSense SDK and provides the USB drivers. The `pyrealsense2` library talks directly to the D435 over the USB connection, enabling both the RGB and IR streams.

### 2. ROS2 middleware

ROS2 provides the message-passing backbone. The publisher node publishes:

* `/realsense/color/image_raw` — 640×480, 30 FPS, `bgr8` color images from the RGB sensor.
* `/realsense/stereo/image_raw` — 640×480, 30 FPS, `mono8` infrared images from the left IR sensor.

The subscriber node listens to both topics. Communication uses DDS under the hood in a publish/subscribe pattern, so any other ROS2 node on the same domain can consume these streams as well.

### 3. Application nodes (Python)

* `publisher.py` uses `pyrealsense2` + `cv_bridge` to convert camera frames into ROS2 `sensor_msgs/msg/Image` messages.
* `subscriber.py` subscribes to those messages, converts them back to OpenCV images, JPEG-encodes them, and serves them via a tiny HTTP server as MJPEG streams.

Conceptually:

* The **RGB stream** represents what you expect from a regular webcam: color frames suitable for visualization, logging, or feeding into vision models.
* The **Stereo / IR stream** is the raw infrared intensity captured by one of the D435’s IR sensors. It:

  * Is monochrome (`mono8`), then converted to 3-channel for display.
  * Is not depth or disparity — just raw IR brightness per pixel.
  * Is extremely useful if you want to reason about lighting, IR patterns, or debug depth issues.
* The **HTTP MJPEG node** is effectively a bridge from ROS2 to the browser:

  * It maintains the latest JPEG-encoded frames in memory.
  * Streams them out as a multipart MJPEG response.
  * The HTML page opens a single MJPEG connection at a time (either RGB or Stereo) and swaps modes by changing the `src` of the `<img>` tag.

You can extend this pattern to publish depth images, point clouds, or other RealSense streams, and expose them via different web front-ends (WebSockets, REST, etc.) while still keeping ROS2 as the messaging backbone.

