#!/usr/bin/env python3
"""
MJPEG camera stream over HTTP for the Spark robot.

Anyone on the same network can open:

    http://<robot-ip>:8080/            live stream in a browser
    http://<robot-ip>:8080/snapshot.jpg   one JPEG frame

No software is needed on the viewing PC (works from any browser).

Prerequisite: the camera driver must be running:
    roslaunch spark_bringup driver_bringup.launch camera_type_tel:=d435 lidar_type_tel:=ydlidar_g6

Run:
    source ~/spark_noetic/devel/setup.bash
    python3 ~/semi/camera_stream.py

Environment:
    PORT=8080        HTTP port
    QUALITY=80       JPEG quality
    FPS=30           maximum stream rate

For a detection overlay, run ~/semi/blue_view.py on the robot screen instead
(or ask for an overlay option in this stream).
"""

import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

PORT = int(os.environ.get("PORT", "8080"))
QUALITY = int(os.environ.get("QUALITY", "80"))
FPS = float(os.environ.get("FPS", "30"))
IMAGE_TOPIC = os.environ.get("IMAGE_TOPIC", "/camera/color/image_raw")
BOUNDARY = "sparkframe"

PAGE = b"""<!DOCTYPE html>
<html><head><meta charset="utf-8"><title>Spark camera</title>
<style>body{background:#111;color:#eee;font-family:sans-serif;text-align:center}
img{max-width:100%;height:auto;border:1px solid #444;margin-top:12px}</style></head>
<body><h2>Spark camera</h2>
<img src="/stream">
<p><a style="color:#8cf" href="/snapshot.jpg">single snapshot</a></p>
</body></html>"""


class FrameBuffer:
    def __init__(self):
        self.lock = threading.Lock()
        self.jpeg = None
        self.bridge = CvBridge()

    def callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            return
        ok, encoded = cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), QUALITY])
        if ok:
            with self.lock:
                self.jpeg = encoded.tobytes()

    def get(self):
        with self.lock:
            return self.jpeg


class Handler(BaseHTTPRequestHandler):
    buffer = None

    def log_message(self, *args):
        pass

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(PAGE)))
            self.end_headers()
            self.wfile.write(PAGE)

        elif self.path == "/snapshot.jpg":
            frame = self.buffer.get()
            if frame is None:
                self.send_error(503, "no frame yet")
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(frame)))
            self.end_headers()
            self.wfile.write(frame)

        elif self.path.startswith("/stream"):
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=" + BOUNDARY)
            self.end_headers()
            try:
                while True:
                    frame = self.buffer.get()
                    if frame is not None:
                        self.wfile.write(b"--" + BOUNDARY.encode() + b"\r\n")
                        self.wfile.write(b"Content-Type: image/jpeg\r\n")
                        self.wfile.write(b"Content-Length: " + str(len(frame)).encode() + b"\r\n\r\n")
                        self.wfile.write(frame)
                        self.wfile.write(b"\r\n")
                        self.wfile.flush()
                    time.sleep(1.0 / FPS)
            except (BrokenPipeError, ConnectionResetError):
                pass
        else:
            self.send_error(404)


def main():
    rospy.init_node("camera_stream", anonymous=True)
    buffer = FrameBuffer()
    rospy.Subscriber(IMAGE_TOPIC, Image, buffer.callback, queue_size=1)

    Handler.buffer = buffer
    ThreadingHTTPServer.allow_reuse_address = True
    server = ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
    print("camera stream (%s): http://<robot-ip>:%d/  (snapshot: /snapshot.jpg)" % (IMAGE_TOPIC, PORT))
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
