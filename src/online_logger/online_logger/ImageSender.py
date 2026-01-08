#!/usr/bin/env python3

import io
import time
import threading
import queue
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

import requests


class ImageSender(Node):
    def __init__(self):
        super().__init__('image_sender')

        try:
            with open("/home/pi/.bin/sek", "r") as f: # scret for image sending
                self.sek = f.read().strip()
        except Exception as e:
            self.get_logger().error(f'Failed to read secret file: {e}')
            self.sek = ''

        # parameters
        self.declare_parameter('min_interval_sec', 2.0)
        self.declare_parameter('verify_ssl', True)

        # read them once
        self.min_interval = float(self.get_parameter('min_interval_sec')
                                  .get_parameter_value().double_value)
        self.verify_ssl = bool(self.get_parameter('verify_ssl')
                               .get_parameter_value().bool_value)

        self.server_url = "https://bix.ovh/cam"

        # small bounded queue to hold latest frame for upload (drop older)
        self._queue: "queue.Queue[Tuple[bytes, Optional[float]]]" = queue.Queue(maxsize=1)

        # rate monotonic
        self._last_send_time = 0.0
        self._lock = threading.Lock()

        # uploader thread control
        self._running = True
        self._uploader_thread = threading.Thread(target=self._uploader_loop, daemon=True)
        self._uploader_thread.start()

        # optional status publisher (recreate if needed)
        self.status_pub = self.create_publisher(String, '/image_sender/status', 10)

        # subscribe
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            qos_profile_sensor_data
        )

        self.get_logger().info(f'ImageSender started. Will POST to {self.server_url} '
                               f'(min_interval={self.min_interval}s, verify_ssl={self.verify_ssl})')

        # create a requests.Session for connection reuse (thread-safe for our use: used only by uploader thread)
        self._session = requests.Session()

    def image_callback(self, msg: CompressedImage):
        # if message empty, ignore
        if not msg.data:
            return

        now = time.monotonic()
        with self._lock:
            if now - self._last_send_time < self.min_interval:
                # too soon
                return
            # reserve the slot immediately so other callbacks will skip until we accept this frame
            self._last_send_time = now

        # zero-copy: pass msg.data directly (it's bytes/bytearray)
        jpeg_data = msg.data  # NO BYTE CALL HERE it slow down everything

        # compute timestamp if available (wall time approximation)
        timestamp = None
        try:
            if msg.header is not None:
                stamp = msg.header.stamp
                # convert ROS time to seconds (if fields present)
                timestamp = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        except Exception:
            timestamp = None

        # Put into the queue: if full, drop the existing item to always keep the newest frame
        try:
            # If queue already full, remove the previous to replace with latest (drop older)
            if self._queue.full():
                try:
                    self._queue.get_nowait()
                except Exception:
                    pass
            self._queue.put_nowait((jpeg_data, timestamp))
        except Exception as e:
            # shouldn't happen often; log at debug level
            self.get_logger().debug(f'Queue put failed: {e}')

    def _uploader_loop(self):
        """Background thread: consumes latest frames from queue and uploads them."""
        session = self._session
        while self._running and rclpy.ok():
            try:
                item = self._queue.get(timeout=0.5)  # wait for a frame
            except queue.Empty:
                continue

            if item is None:
                continue
            jpeg_bytes, timestamp = item

            # pass without copy
            files = {
                'image': ('frame.jpg', jpeg_bytes, 'image/jpeg'),
            }
            data = {'popop': self.sek}
            if timestamp is not None:
                # include high precision timestamp
                data['timestamp'] = f'{timestamp:.6f}'

            try:
                # perform HTTP POST (single Session reused)
                resp = session.post(self.server_url, files=files, data=data,
                                    timeout=8, verify=self.verify_ssl)
                if resp.status_code == 200:
                    self.get_logger().info('Image uploaded successfully '
                                           f'(response {len(resp.content)} bytes)')
                else:
                    self.get_logger().warning(f'Upload failed: HTTP {resp.status_code}')
            except requests.RequestException as e:
                # common network exceptions
                self.get_logger().warning(f'Network error during upload: {e}')
            except Exception as e:
                # unexpected exception
                self.get_logger().error(f'Unexpected exception in uploader: {e}')
            finally:
                # ensure we mark task done (queue.get() used without task_done here)
                pass

    def close(self):
        """Stop uploader thread and clean up."""
        self.get_logger().info('Shutting down uploader thread...')
        self._running = False
        # clear queue so uploader won't block long
        try:
            while not self._queue.empty():
                self._queue.get_nowait()
        except Exception:
            pass
        # join thread with timeout to avoid blocking shutdown indefinitely
        if self._uploader_thread.is_alive():
            self._uploader_thread.join(timeout=2.0)
        try:
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
        node.get_logger().info('Shutting down ImageSender node...')
        # stop uploader
        try:
            node.close()
        except Exception:
            pass
        # destroy and shutdown rclpy
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()