#!/usr/bin/env python3
import sys
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class EchoOnce(Node):
    def __init__(self, topic: str):
        super().__init__('echo_json_once')
        self._done = False
        self._sub = self.create_subscription(String, topic, self._on_msg, 10)

    def _on_msg(self, msg: String) -> None:
        try:
            obj = json.loads(msg.data)
        except Exception:
            obj = {'raw': msg.data}
        print(json.dumps(obj, separators=(',', ':'), ensure_ascii=False))
        self._done = True


def main() -> int:
    if len(sys.argv) < 2:
        print('Usage: echo_json_once.py <topic>')
        return 2
    topic = sys.argv[1]
    rclpy.init()
    node = EchoOnce(topic)
    try:
        while rclpy.ok() and not node._done:
            rclpy.spin_once(node, timeout_sec=0.5)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
