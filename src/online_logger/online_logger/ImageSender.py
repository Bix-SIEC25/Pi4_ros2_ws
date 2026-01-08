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

        # read secret token once
        try:
            with open("/home/pi/.bin/sek", "r") as f:
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

        # upload target
        self.server_url = "https://bix.ovh/cam"

        # small bounded queue to hold latest frame for upload (drop older)
        self._queue: "queue.Queue[Tuple[bytes, Optional[float]]]" = queue.Queue(maxsize=1)

        # last send time (monotonic)
        self._last_send_time = 0.0
        self._lock = threading.Lock()

        # create a requests.Session for connection reuse BEFORE starting the uploader thread
        # (session will be used only by the uploader thread)
        try:
            self._session = requests.Session()
        except Exception as e:
            self.get_logger().warning(f'Failed to create requests Session: {e}')
            self._session = None

        # uploader thread control
        self._running = True
        self._uploader_thread = threading.Thread(target=self._uploader_loop, daemon=True)
        self._uploader_thread.start()

        # optional status publisher
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

    def image_callback(self, msg: CompressedImage):
        # if message empty, ignore
        if not msg.data:
            return

        now = time.monotonic()
        with self._lock:
            if now - self._last_send_time < self.min_interval:
                # too soon — skip (do no heavy work)
                return
            # reserve the slot immediately so other callbacks will skip until we accept this frame
            self._last_send_time = now

        # zero-copy: pass msg.data directly (it's bytes/bytearray)
        jpeg_data = msg.data  # do NOT call bytes(...) here

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
        # defensive: ensure we have a session even if __init__ didn't set it for some reason
        session = getattr(self, '_session', None)
        if session is None:
            try:
                session = requests.Session()
                self._session = session
            except Exception as e:
                self.get_logger().warning(f'Uploader thread could not create Session(): {e}')
                session = None

        while self._running and rclpy.ok():
            try:
                item = self._queue.get(timeout=0.5)  # wait for a frame
            except queue.Empty:
                continue

            if item is None:
                continue
            jpeg_bytes, timestamp = item

            # Build multipart form: we pass jpeg_bytes directly (no copy)
            files = {
                'image': ('frame.jpg', jpeg_bytes, 'image/jpeg'),
            }
            data = {'popop': self.sek}
            if timestamp is not None:
                # include high precision timestamp
                data['timestamp'] = f'{timestamp:.6f}'

            try:
                # perform HTTP POST (single Session reused)
                if session is None:
                    # fallback to simple post if session creation failed earlier
                    resp = requests.post(self.server_url, files=files, data=data,
                                         timeout=8, verify=self.verify_ssl)
                else:
                    resp = session.post(self.server_url, files=files, data=data,
                                        timeout=8, verify=self.verify_ssl)

                if resp.status_code == 200:
                    self.get_logger().info('Image uploaded successfully '
                                           f'(response {len(resp.content)} bytes)')
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
                # common network exceptions
                self.get_logger().warning(f'Network error during upload: {e}')
                try:
                    self.status_pub.publish(String(data='upload_exception'))
                except Exception:
                    pass
            except Exception as e:
                # unexpected exception
                self.get_logger().error(f'Unexpected exception in uploader: {e}')
                try:
                    self.status_pub.publish(String(data='upload_exception'))
                except Exception:
                    pass
            finally:
                # continue to next item
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
            if getattr(self, '_session', None) is not None:
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
