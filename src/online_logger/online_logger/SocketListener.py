#!/usr/bin/env python3
"""
ws_music_bridge.py

Connect to wss://server.org/ws, send "subscribe:horn" on each connection,
and when receiving the message "horn" call the /music_play service:
  ros2 service call /music_play audio_common_msgs/srv/MusicPlay "{audio: 'horn'}"

Requirements:
  pip3 install websockets
"""

import threading
import asyncio
import time
import sys
from typing import Optional

import rclpy
from rclpy.node import Node

# Service type
from audio_common_msgs.srv import MusicPlay

# websocket client lib
try:
    import websockets
except Exception as e:
    websockets = None

WS_URL = "wss://magictintin.fr/ws"
SUBSCRIBE_MSG = "bix/wristband:ping"
EXPECTED_MESSAGE = "new fall"


class WebsocketMusicBridge(Node):
    def __init__(
        self,
        ws_url: str = WS_URL,
        subscribe_message: str = SUBSCRIBE_MSG,
        expected_message: str = EXPECTED_MESSAGE,
    ):
        super().__init__("ws_music_bridge")
        self.ws_url = ws_url
        self.subscribe_message = subscribe_message
        self.expected_message = expected_message

        if websockets is None:
            self.get_logger().error(
                "Python package 'websockets' not found. Install with: pip3 install websockets"
            )
            raise RuntimeError("missing dependency: websockets")

        # ROS service client
        self._music_client = self.create_client(MusicPlay, "/music_play")
        # we don't block here waiting for service; calls will wait or log if not available

        # Asyncio loop & thread for websocket
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._ws_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()

        # Start background thread running asyncio loop
        self._start_background_loop()

        self.get_logger().info(f"WebsocketMusicBridge initialized, target: {self.ws_url}")

    def _start_background_loop(self):
        """asyncio loop in a background daemon thread and websocket main coroutine."""
        self._loop = asyncio.new_event_loop()
        self._ws_thread = threading.Thread(target=self._run_loop, daemon=True)
        self._ws_thread.start()

    def _run_loop(self):
        asyncio.set_event_loop(self._loop)
        # Run the ws_main until cancelled
        try:
            self._loop.run_until_complete(self._ws_main())
        except Exception as e:
            # If loop stops due to exception, log from node (thread-safe)
            self.get_logger().error(f"Websocket thread exited with exception: {e}")
        finally:
            # cleanup if needed
            pending = asyncio.all_tasks(loop=self._loop)
            for t in pending:
                t.cancel()
            try:
                self._loop.run_until_complete(asyncio.gather(*pending, return_exceptions=True))
            except Exception:
                pass
            self._loop.close()

    async def _ws_main(self):
        """main reconnection loop. On each connection send subscribe_message and process incoming messages."""
        backoff = 1.0  # seconds
        max_backoff = 30.0
        while rclpy.ok() and not self._stop_event.is_set():
            try:
                self.get_logger().info(f"Attempting websocket connect to {self.ws_url}")
                # timeout and close handling set here; adjust ping_interval/ping_timeout if desired
                async with websockets.connect(self.ws_url, ping_interval=20, ping_timeout=10) as ws:
                    self.get_logger().info("Websocket connected, sending subscribe message")
                    try:
                        await ws.send(self.subscribe_message)
                    except Exception as e:
                        self.get_logger().warn(f"Failed to send subscribe message: {e}")

                    # reset backoff on successful connect
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
                            # Make a ROS service call (non-blocking)
                            self._call_music_play(msg)
                        # else: ignore or extend for other messages
                    # if we exit the async for, connection closed gracefully
                    self.get_logger().warn("Websocket connection closed, will attempt reconnect")
            except (websockets.ConnectionClosedOK, websockets.ConnectionClosedError) as e:
                self.get_logger().warn(f"Websocket disconnected: {e}")
            except Exception as e:
                # network error, DNS, TLS, handshake error, etc.
                self.get_logger().error(f"Websocket connection error: {e}")

            # If we reach here, we're disconnected -> backoff & retry unless stopping
            if self._stop_event.is_set() or not rclpy.ok():
                break
            self.get_logger().info(f"Reconnect in {backoff:.1f}s...")
            await asyncio.sleep(backoff)
            backoff = min(backoff * 2.0, max_backoff)

        self.get_logger().info("Websocket main coroutine exiting")

    def _call_music_play(self, audio_name: str):
        """
        Call the /music_play service asynchronously.
        This is safe to call from the background (non-ROS) thread.
        """
        if not self._music_client.wait_for_service(timeout_sec=1.0):
            # service not available right now
            self.get_logger().warn("/music_play service not available (timeout), skipping play request")
            return

        req = MusicPlay.Request()
        req.audio = audio_name

        # call_async returns a future handled by rclpy executor (ensure rclpy.spin is running)
        future = self._music_client.call_async(req)

        # attach callback to log result when ready
        def _on_response(fut):
            try:
                resp = fut.result()
                # MusicPlay usually returns an empty response or success flag depending on service definition
                self.get_logger().info(f"MusicPlay service call finished for '{audio_name}'")
            except Exception as e:
                self.get_logger().error(f"MusicPlay service call failed: {e}")

        future.add_done_callback(lambda fut: _on_response(fut))

    def destroy_node(self):
        # signal background websocket loop to stop and close the asyncio loop
        self.get_logger().info("Stopping websocket thread...")
        self._stop_event.set()
        # attempt graceful loop stop
        if self._loop is not None:
            try:
                # schedule loop shutdown
                self._loop.call_soon_threadsafe(self._loop.stop)
            except Exception:
                pass
        # join thread
        if self._ws_thread is not None:
            self._ws_thread.join(timeout=2.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = None
    try:
        node = WebsocketMusicBridge()
        # spin() must run to process service futures and keep the node alive
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        # ensure errors are reported
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
