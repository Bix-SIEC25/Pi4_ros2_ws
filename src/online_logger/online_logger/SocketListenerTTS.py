#!/usr/bin/env python3
import threading
import asyncio
import time
import sys
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

# action type
from audio_common_msgs.action import TTS

# websocket client lib
try:
    import websockets
except Exception as e:
    websockets = None

WS_URL = "wss://magictintin.fr/ws"
SUBSCRIBE_MSG = "bix/wristband:ping"
EXPECTED_MESSAGE = "new fall"


class WebsocketTTSBridge(Node):
    def __init__(
        self,
        ws_url: str = WS_URL,
        subscribe_message: str = SUBSCRIBE_MSG,
        expected_message: str = EXPECTED_MESSAGE,
    ):
        super().__init__("ws_tts_bridge")
        self.ws_url = ws_url
        self.subscribe_message = subscribe_message
        self.expected_message = expected_message

        if websockets is None:
            self.get_logger().error(
                "Python package 'websockets' not found. Install with: pip3 install websockets"
            )
            raise RuntimeError("missing dependency: websockets")

        # ROS action client for TTS
        # the action server name in your example is '/say'
        self._tts_client = ActionClient(self, TTS, "/say")

        # Asyncio loop & thread for websocket
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._ws_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()

        # Start background thread running asyncio loop
        self._start_background_loop()

        self.get_logger().info(f"WebsocketTTSBridge initialized, target: {self.ws_url}")

    def _start_background_loop(self):
        """Create an asyncio loop in a background daemon thread and run the websocket main coroutine."""
        self._loop = asyncio.new_event_loop()
        self._ws_thread = threading.Thread(target=self._run_loop, daemon=True)
        self._ws_thread.start()

    def _run_loop(self):
        """Thread target: set and run asyncio loop."""
        asyncio.set_event_loop(self._loop)
        try:
            self._loop.run_until_complete(self._ws_main())
        except Exception as e:
            self.get_logger().error(f"Websocket thread exited with exception: {e}")
        finally:
            pending = asyncio.all_tasks(loop=self._loop)
            for t in pending:
                t.cancel()
            try:
                self._loop.run_until_complete(asyncio.gather(*pending, return_exceptions=True))
            except Exception:
                pass
            self._loop.close()

    async def _ws_main(self):
        """Main reconnection loop. On each connection send subscribe_message and process incoming messages."""
        backoff = 1.0  # seconds
        max_backoff = 30.0
        while rclpy.ok() and not self._stop_event.is_set():
            try:
                self.get_logger().info(f"Attempting websocket connect to {self.ws_url}")
                async with websockets.connect(self.ws_url, ping_interval=20, ping_timeout=10) as ws:
                    self.get_logger().info("Websocket connected, sending subscribe message")
                    try:
                        await ws.send(self.subscribe_message)
                    except Exception as e:
                        self.get_logger().warn(f"Failed to send subscribe message: {e}")

                    backoff = 1.0

                    # Listen for messages until the socket closes or we are asked to stop
                    async for raw_msg in ws:
                        if raw_msg is None:
                            break
                        msg = raw_msg.strip()
                        if not msg:
                            continue
                        self.get_logger().info(f"Websocket received: {msg}")
                        # react to the expected message (exact match)
                        if msg == self.expected_message:
                            # Send TTS action goal (non-blocking)
                            self._send_tts_goal("new fall")
                    # connection closed gracefully
                    self.get_logger().warn("Websocket connection closed, will attempt reconnect")
            except (websockets.ConnectionClosedOK, websockets.ConnectionClosedError) as e:
                self.get_logger().warn(f"Websocket disconnected: {e}")
            except Exception as e:
                self.get_logger().error(f"Websocket connection error: {e}")

            # If we reach here, we're disconnected -> backoff & retry unless stopping
            if self._stop_event.is_set() or not rclpy.ok():
                break
            self.get_logger().info(f"Reconnect in {backoff:.1f}s...")
            await asyncio.sleep(backoff)
            backoff = min(backoff * 2.0, max_backoff)

        self.get_logger().info("Websocket main coroutine exiting")

    def _send_tts_goal(self, text: str):
        """
        Send a TTS goal to the /say action server.
        This can be called from the websocket thread safely.
        """
        # ensure the action server is available (short timeout)
        if not self._tts_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("/say action server not available (timeout), skipping TTS request")
            return

        goal_msg = TTS.Goal()
        goal_msg.text = text

        # send_goal_async returns a future (rclpy Future)
        send_goal_future = self._tts_client.send_goal_async(goal_msg)

        # attach a callback to handle goal acceptance / result
        def goal_response_callback(fut):
            try:
                goal_handle = fut.result()
            except Exception as e:
                self.get_logger().error(f"TTS send_goal failed: {e}")
                return

            if not goal_handle.accepted:
                self.get_logger().warn("TTS goal rejected by server")
                return

            self.get_logger().info("TTS goal accepted, waiting for result...")
            result_future = goal_handle.get_result_async()

            def result_callback(res_fut):
                try:
                    result = res_fut.result().result
                    # The TTS action result content depends on action definition; log completion
                    self.get_logger().info("TTS action finished")
                except Exception as e:
                    self.get_logger().error(f"TTS action failed: {e}")

            result_future.add_done_callback(result_callback)

        send_goal_future.add_done_callback(goal_response_callback)

    def destroy_node(self):
        # signal background websocket loop to stop and close the asyncio loop
        self.get_logger().info("Stopping websocket thread...")
        self._stop_event.set()
        if self._loop is not None:
            try:
                self._loop.call_soon_threadsafe(self._loop.stop)
            except Exception:
                pass
        if self._ws_thread is not None:
            self._ws_thread.join(timeout=2.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = None
    try:
        node = WebsocketTTSBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if node is not None:
            node.get_logger().error(f"Fatal error: {e}")
        else:
            print(f"Fatal error: {e}", file=sys.stderr)
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        rclpy.shutdown()


if __name__ == "__main__":
    main()
