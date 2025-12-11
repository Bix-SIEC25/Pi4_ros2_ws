#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import concurrent.futures
import urllib.parse
import urllib.request
import ssl

try:
    import requests  # type: ignore
    HAS_REQUESTS = True
except Exception:
    HAS_REQUESTS = False

from interfaces.msg import LogEntry

LEVEL_NAMES = {
    4: "MICASEND",
    3: "DEBUG",
    2: "TRACE",
    1: "WARN",
    0: "ERROR"
}

MS_SERVER_BASE = "https://micasend.magictintin.fr/msg.php"
SERVER_BASE = "https://bix.ovh/add_log"
HTTP_TIMEOUT = 5  # seconds


class LoggerSubscriber(Node):

    def __init__(self, topic_name: str = '/logger'):
        super().__init__('LoggerSubscriber')
        qos = QoSProfile(depth=10)
        self.subscription = self.create_subscription(
            LogEntry,
            topic_name,
            self.listener_callback,
            qos)
        self.subscription  # avoid unused var warning

        # thread pool to make non bloquant request
        self._executor = concurrent.futures.ThreadPoolExecutor(max_workers=4)

        # SSL context used by urllib fallback
        self._ssl_context = ssl.create_default_context()

        self.get_logger().info(f"LoggerSubscriber started, subscribing to: {topic_name}")

    def listener_callback(self, msg: LogEntry):
        level_name = LEVEL_NAMES.get(msg.level, f"LEVEL({msg.level})")
        self.get_logger().info(f"Received log [{level_name}] {msg.sender}: {msg.message}")

        # Schedule HTTP sending in a thread pool to avoid blocking callback
        if int(msg.level) == 4:
            try:
                self._executor.submit(self._send_to_ms_server, str(msg.sender), str(msg.message))
            except Exception as e:
                self.get_logger().error(f"Failed to schedule send_to_ms_server: {e}")
        else:
            try:
                self._executor.submit(self._send_to_server, str(msg.sender), int(msg.level), str(msg.message))
            except Exception as e:
                self.get_logger().error(f"Failed to schedule send_to_server: {e}")

    def _send_to_ms_server(self, sender: str, message: str) -> None:
        # Build params and URL-encode them
        params = {'sender': sender, 'message': message}
        query = urllib.parse.urlencode(params, safe='')
        url = f"{MS_SERVER_BASE}?{query}"

        try:
            if HAS_REQUESTS:
                # requests package
                resp = requests.get(MS_SERVER_BASE, params=params, timeout=HTTP_TIMEOUT)
                status = resp.status_code
                text_preview = resp.text[:200]  # crop
                self.get_logger().info(f"Sent to ms server ({status})")
            else:
                # urllib fallback
                req = urllib.request.Request(url, method='GET')
                with urllib.request.urlopen(req, timeout=HTTP_TIMEOUT, context=self._ssl_context) as resp:
                    body = resp.read(200)  #crop
                    status = resp.getcode()
                    self.get_logger().info(f"Sent to ms server ({status})")
        except Exception as e:
            self.get_logger().error(f"Error sending log to server for sender='{sender}': {e}")

    def _send_to_server(self, sender: str, level: int, message: str) -> None:
        # Build params and URL-encode them
        params = {'sender': sender, 'type': level, 'msg': message}
        query = urllib.parse.urlencode(params, safe='')
        url = f"{SERVER_BASE}?{query}"

        try:
            if HAS_REQUESTS:
                # requests package
                resp = requests.get(SERVER_BASE, params=params, timeout=HTTP_TIMEOUT)
                status = resp.status_code
                text_preview = resp.text[:200]  # crop
                self.get_logger().info(f"Sent to server ({status}) - preview: {text_preview}")
            else:
                # urllib fallback
                req = urllib.request.Request(url, method='GET')
                with urllib.request.urlopen(req, timeout=HTTP_TIMEOUT, context=self._ssl_context) as resp:
                    body = resp.read(200)  #crop
                    status = resp.getcode()
                    self.get_logger().info(f"Sent to server ({status}) - preview: {body!r}")
        except Exception as e:
            self.get_logger().error(f"Error sending log to server for sender='{sender}': {e}")
            
    def destroy_node(self):
        # clean up executor
        try:
            self._executor.shutdown(wait=False)
        except Exception:
            pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    topic = '/logger'
    node = LoggerSubscriber(topic_name=topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
