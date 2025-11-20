#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

# === CHANGE THIS to the package that contains your LogEntry.msg ===
# If your msg is defined in package named "my_logger_msgs", leave as below.
# If it's in the same package, use: from <your_package_name>.msg import LogEntry
from interfaces.msg import LogEntry

# Map numeric levels to readable names
LEVEL_NAMES = {
    0: "TRACE",
    1: "DEBUG",
    2: "ERROR"
}


class LoggerSubscriber(Node):

    def __init__(self, topic_name: str = '/logger'):
        super().__init__('LoggerSubscriber')
        qos = QoSProfile(depth=10)
        # subscribe to the topic carrying LogEntry messages
        self.subscription = self.create_subscription(
            LogEntry,
            topic_name,
            self.listener_callback,
            qos)
        self.subscription  # avoid unused var warning

    def listener_callback(self, msg: LogEntry):
        # convert numeric level to a name (fallback to LEVEL(n) if unknown)
        level_name = LEVEL_NAMES.get(msg.level, f"LEVEL({msg.level})")
        # msg.sender and msg.message are strings as defined in LogEntry.msg
        self.get_logger().info(f"[{level_name}] {msg.sender}: {msg.message}")


def main(args=None):
    rclpy.init(args=args)

    # You can change the topic name here if you used a different one (e.g. 'Logger')
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
