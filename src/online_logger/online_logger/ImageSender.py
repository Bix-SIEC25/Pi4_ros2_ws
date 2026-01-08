#!/usr/bin/env python3
import time
import threading
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import requests


class ImageSender(Node):
    def __init__(self):
        super().__init__('image_sender')

        # secret token
        try:
            with open("/home/pi/.bin/sek", "r") as f:
                self.sek = f.read().strip()
        except Exception as e:
            self.get_logger().error(f'Failed to read secret file: {e}')
            self.sek = ''

        self.declare_parameter('min_interval_sec', 2.0)
        self.declare_parameter('verify_ssl', True)
        self.declare_parameter('timeout_sec', 5.0)

        self.min_interval = float(self.get_parameter('min_interval_sec')
                                  .get_parameter_value().double_value)
        self.verify_ssl = bool(self.get_parameter('verify_ssl')
                               .get_parameter_value().bool_value)
        self.timeout_sec = float(self.get_parameter('timeout_sec')
                                 .get_parameter_value().double_value)

        self.server_url = "https://bix.ovh/cam"

        # latest-frame slot (instead of Queue)
        self._latest: Optional[Tuple[bytes, Optional[float]]] = None
        self._latest_lock = threading.Lock()
        self._event = threading.Event()

        # uploader timing
        self._last_send_time = 0.0

        # requests session (used only in uploader thread)
        self._session = None
        try:
            self._session = requests.Session()
        except Exception as e:
            self.get_logger().warning(f'Failed to create requests Session: {e}')

        self._running = True
        self._uploader_thread = threading.Thread(target=self._uploader_loop, daemon=True)
        self._uploader_thread.start()

        self.status_pub = self.create_publisher(String, '/image_sender/status', 10)

        # QoS: keep only last message, best effort (light)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            qos
        )

        self.get_logger().info(
            f'ImageSender started. POST {self.server_url} '
            f'(min_interval={self.min_interval}s, verify_ssl={self.verify_ssl}, timeout={self.timeout_sec}s)'
        )

    def image_callback(self, msg: CompressedImage):
        if not msg.data:
            return

        timestamp = None
        try:
            stamp = msg.header.stamp
            timestamp = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        except Exception:
            pass

        # just store latest frame (very cheap)
        with self._latest_lock:
            self._latest = (msg.data, timestamp)
        self._event.set()

    def _uploader_loop(self):
        session = self._session

        while self._running and rclpy.ok():
            # wait until a new frame arrives (wake up rarely)
            if not self._event.wait(timeout=0.5):
                continue

            # throttle here (not in callback)
            now = time.monotonic()
            if now - self._last_send_time < self.min_interval:
                # we can clear the event and wait again; latest frame remains stored
                self._event.clear()
                continue

            # grab the latest frame snapshot
            with self._latest_lock:
                item = self._latest
            self._event.clear()

            if not item:
                continue

            jpeg_bytes, timestamp = item

            files = {'image': ('frame.jpg', jpeg_bytes, 'image/jpeg')}
            data = {'popop': self.sek}
            if timestamp is not None:
                data['timestamp'] = f'{timestamp:.6f}'

            try:
                self._last_send_time = now
                if session is None:
                    resp = requests.post(self.server_url, files=files, data=data,
                                         timeout=self.timeout_sec, verify=self.verify_ssl)
                else:
                    resp = session.post(self.server_url, files=files, data=data,
                                        timeout=self.timeout_sec, verify=self.verify_ssl)

                if resp.status_code == 200:
                    # log less noisy: debug instead of info
                    self.get_logger().debug('Image uploaded successfully')
                    try:
                        self.status_pub.publish(String(data='uploaded'))
                    except Exception:
                        pass
                else:
                    self.get_logger().warning(f'Upload failed: HTTP {resp.status_code}')
                    try:
                        self.status_pub.publish(String(data=f'upload_failed:{resp.status_code}'))
                    except Exception:
                        pass

            except requests.RequestException as e:
                self.get_logger().warning(f'Network error during upload: {e}')
                try:
                    self.status_pub.publish(String(data='upload_exception'))
                except Exception:
                    pass
            except Exception as e:
                self.get_logger().error(f'Unexpected exception in uploader: {e}')
                try:
                    self.status_pub.publish(String(data='upload_exception'))
                except Exception:
                    pass

    def close(self):
        self.get_logger().info('Shutting down uploader thread...')
        self._running = False
        self._event.set()
        if self._uploader_thread.is_alive():
            self._uploader_thread.join(timeout=2.0)
        try:
            if self._session is not None:
                self._session.close()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = ImageSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.close()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()
