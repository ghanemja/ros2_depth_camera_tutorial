#!/usr/bin/env python3
"""
ROS2 node that:
- Subscribes to:
    /realsense/color/image_raw
    /realsense/stereo/image_raw
    /realsense/depth_vis
- Streams all three as MJPEG over HTTP:
    /rgb.mjpg
    /stereo.mjpg
    /depth.mjpg
- Serves an HTML page at http://<this-machine-ip>:8000/
  with a toggle to switch between RGB, Stereo IR, and Depth.
"""

import threading
import time
import sys
from typing import Optional

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse
import cv2
import numpy as np

# Global frame storage
_latest_rgb_jpeg_lock = threading.Lock()
_latest_rgb_jpeg: Optional[bytes] = None

_latest_stereo_jpeg_lock = threading.Lock()
_latest_stereo_jpeg: Optional[bytes] = None

_latest_depth_jpeg_lock = threading.Lock()
_latest_depth_jpeg: Optional[bytes] = None


class RealSenseHttpSubscriber(Node):
    def __init__(self):
        super().__init__('realsense_http_subscriber')

        # Fixed topics to match your publisher
        self.color_topic = "/realsense/color/image_raw"
        self.stereo_topic = "/realsense/stereo/image_raw"
        self.depth_topic = "/realsense/depth_vis"

        self.get_logger().info(f"Subscribing to RGB:    {self.color_topic}")
        self.get_logger().info(f"Subscribing to Stereo: {self.stereo_topic}")
        self.get_logger().info(f"Subscribing to Depth:  {self.depth_topic}")

        self.bridge = CvBridge()

        # Subscribers
        self.rgb_sub = self.create_subscription(
            Image, self.color_topic, self.rgb_callback, 10
        )
        self.stereo_sub = self.create_subscription(
            Image, self.stereo_topic, self.stereo_callback, 10
        )
        self.depth_sub = self.create_subscription(
            Image, self.depth_topic, self.depth_callback, 10
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

    def depth_callback(self, msg: Image):
        global _latest_depth_jpeg

        try:
            # Publisher already sends BGR8 colorized depth
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warning(f"[DEPTH] Conversion error: {e}")
            return

        ok, jpeg = cv2.imencode('.jpg', img)
        if not ok:
            self.get_logger().warning("[DEPTH] JPEG encoding failed")
            return

        with _latest_depth_jpeg_lock:
            _latest_depth_jpeg = jpeg.tobytes()

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
        path = urlparse(self.path).path
        sys.stderr.write(f"[HTTP] Raw path: {self.path}, parsed path: {path}\n")

        if path in ("/", "/index.html"):
            self._serve_html()
        elif path == "/rgb.mjpg":
            self._stream(kind="rgb")
        elif path == "/stereo.mjpg":
            self._stream(kind="stereo")
        elif path == "/depth.mjpg":
            self._stream(kind="depth")
        else:
            # favicon.ico etc. will end up here
            self.send_error(404, "Not Found")

    def _serve_html(self):
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
    <button id="stereoBtn">Stereo IR</button>
    <button id="depthBtn">Depth</button>
  </div>

  <!-- Single stream element -->
  <img id="stream" src="" alt="Camera stream">
  <div id="modeLabel">Mode: RGB</div>

<script>
  const img = document.getElementById('stream');
  const rgbBtn = document.getElementById('rgbBtn');
  const stereoBtn = document.getElementById('stereoBtn');
  const depthBtn = document.getElementById('depthBtn');
  const modeLabel = document.getElementById('modeLabel');

  let currentMode = null;

  function startStream(mode) {
    let base;
    if (mode === 'rgb') {
      base = '/rgb.mjpg';
    } else if (mode === 'stereo') {
      base = '/stereo.mjpg';
    } else if (mode === 'depth') {
      base = '/depth.mjpg';
    } else {
      return;
    }

    const url = base + '?t=' + Date.now(); // cache-buster

    // Close previous connection by clearing src first
    img.src = '';
    setTimeout(() => {
      img.src = url;
    }, 50);

    currentMode = mode;

    if (mode === 'rgb') {
      modeLabel.textContent = 'Mode: RGB';
      rgbBtn.classList.add('active');
      stereoBtn.classList.remove('active');
      depthBtn.classList.remove('active');
    } else if (mode === 'stereo') {
      modeLabel.textContent = 'Mode: Stereo IR';
      stereoBtn.classList.add('active');
      rgbBtn.classList.remove('active');
      depthBtn.classList.remove('active');
    } else if (mode === 'depth') {
      modeLabel.textContent = 'Mode: Depth';
      depthBtn.classList.add('active');
      rgbBtn.classList.remove('active');
      stereoBtn.classList.remove('active');
    }
  }

  rgbBtn.addEventListener('click', () => {
    if (currentMode !== 'rgb') startStream('rgb');
  });

  stereoBtn.addEventListener('click', () => {
    if (currentMode !== 'stereo') startStream('stereo');
  });

  depthBtn.addEventListener('click', () => {
    if (currentMode !== 'depth') startStream('depth');
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

    def _stream(self, kind: str):
        global _latest_rgb_jpeg, _latest_stereo_jpeg, _latest_depth_jpeg

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
                elif kind == "stereo":
                    with _latest_stereo_jpeg_lock:
                        frame = _latest_stereo_jpeg
                elif kind == "depth":
                    with _latest_depth_jpeg_lock:
                        frame = _latest_depth_jpeg
                else:
                    frame = None

                if frame is None:
                    time.sleep(0.05)
                    continue

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


def start_http_server(host="0.0.0.0", port=8000) -> ThreadingHTTPServer:
    server = ThreadingHTTPServer((host, port), StreamRequestHandler)

    def _serve():
        try:
            server.serve_forever()
        except Exception as e:
            sys.stderr.write(f"[HTTP] Server error: {e}\n")

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


if __name__ == "__main__":
    main()

