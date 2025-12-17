#!/usr/bin/env python3

import time
import threading
from concurrent.futures import ThreadPoolExecutor

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

import requests

class ImageSender(Node):
    def __init__(self):
        super().__init__('image_sender')

        self.declare_parameter('min_interval_sec', 2)  # minimum seconds between HTTP uploads (rate limiting)
        self.declare_parameter('verify_ssl', True)     # verify SSL certificates by default

        self.server_url = "https://bix.ovh/cam"
        self.min_interval = self.get_parameter('min_interval_sec').get_parameter_value().double_value
        self.verify_ssl = self.get_parameter('verify_ssl').get_parameter_value().bool_value
        self.max_workers = 2

        # self.status_pub = self.create_publisher(String, '/...', 10)

        self.http_executor = ThreadPoolExecutor(max_workers=self.max_workers)

        # keep track of last send time
        self._last_send_time = 0.0
        self._lock = threading.Lock()

        # sub to compressed image topic
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            qos_profile_sensor_data
        )

        self.get_logger().info(f'ImageSender started. Will POST to {self.server_url}')

    def image_callback(self, msg: CompressedImage):
        # Basic sanity
        if not msg.data:
            return

        now = time.time()
        with self._lock:
            if now - self._last_send_time < self.min_interval:
                # skip frame for rate limiting
                return
            self._last_send_time = now

        # Optionally include some metadata (for example: frame header timestamp)
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 if msg.header is not None else None
        # if you have resident id elsewhere, set resident_id accordingly; left None here.

        jpeg_bytes = bytes(msg.data) # this is already JPEG-compressed bytes for CompressedImage
        self.http_executor.submit(self._post_image, jpeg_bytes, timestamp)

    def _post_image(self, jpeg_bytes: bytes, timestamp):
        try:
            files = {
                # 'image' is the form-field name; the filename is optional but useful
                'image': ('frame.jpg', jpeg_bytes, 'image/jpeg'),
            }
            data = {}
            if timestamp is not None:
                data['timestamp'] = f'{timestamp:.6f}'
                
            self.get_logger().debug('Posting image to server...')
            resp = requests.post(self.server_url, files=files, data=data, timeout=8, verify=self.verify_ssl)
            if resp.status_code == 200:
                self.get_logger().info(f'Image uploaded successfully (server response length {len(resp.content)})')
                try:
                    self.status_pub.publish(String(data='uploaded'))
                except Exception:
                    pass
            else:
                self.get_logger().warn(f'Upload failed: HTTP {resp.status_code} - {resp.text[:200]}')
                try:
                    self.status_pub.publish(String(data=f'upload_failed:{resp.status_code}'))
                except Exception:
                    pass
        except Exception as e:
            self.get_logger().error(f'Exception while uploading image: {e}')
            try:
                self.status_pub.publish(String(data='upload_exception'))
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
        node.get_logger().info('Shutting down ImageSender...')
        try:
            node.http_executor.shutdown(wait=True)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()