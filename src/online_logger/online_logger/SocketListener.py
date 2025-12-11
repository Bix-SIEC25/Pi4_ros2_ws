#!/usr/bin/env python3

import math
import threading
import asyncio
import time
import sys
from typing import Optional

from tf2_ros import Buffer, TransformListener
# import tf_transformations

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

# Service type
from audio_common_msgs.srv import MusicPlay
from audio_common_msgs.action import TTS


from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool


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
    4: "TRACE",
    3: "DEBUG",
    2: "TRACE",
    1: "WARN",
    0: "ERROR"
}

SERVER_BASE = "https://bix.ovh/add_log"
HTTP_TIMEOUT = 5  # seconds

# websocket client lib
try:
    import websockets
except Exception as e:
    websockets = None

WS_URL = "wss://magictintin.fr/ws"
SUBSCRIBE_MSGS = ["bix/wristband:ping", "micasend:ping", "bix/goto:ping"] #, "bix/fall_alert:ping"
FALL_MESSAGE = "new fall"
HORN_MESSAGE = "horn"
ALRT_MESSAGE = "fall>"
GOTO_MESSAGE = "goto"
STOP_MESSAGE = "stopgoto"

def extract_coordinates(input_str):
    if input_str.startswith("goto"):
        input_str = input_str[4:]

    x_str, y_str = input_str.split('|')
    x = float(x_str)
    y = float(y_str)

    return x, y

class SocketListener(Node):
    def __init__(
        self,
        ws_url: str = WS_URL,
        subscribe_messages = SUBSCRIBE_MSGS,
        fall_message: str = FALL_MESSAGE,
        horn_message: str = HORN_MESSAGE,
        goto_message: str = GOTO_MESSAGE,
        alrt_message: str = ALRT_MESSAGE,
        stop_message: str = STOP_MESSAGE,
    ):
        super().__init__("socket_listener")
        self.ws_url = ws_url
        self.subscribe_messages = subscribe_messages
        self.fall_message = fall_message
        self.horn_message = horn_message
        self.goto_message = goto_message
        self.alrt_message = alrt_message
        self.stop_message = stop_message

        if websockets is None:
            self.get_logger().error(
                "Python package 'websockets' not found. Install with: pip3 install websockets"
            )
            raise RuntimeError("missing dependency: websockets")

        # publish on this topic
        self.arrived_pub = self.create_publisher(Bool, '/car_arrived_to_fall', 10)
        
        # ROS service clients
        self._music_client = self.create_client(MusicPlay, "/music_play")
        self._tts_client = ActionClient(self, TTS, "/say")
        # nav2 service client
        self._client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        # get the current position
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # stop at arrival
        self.arrival_distance = 0.10   # 10 cm threshold
        self._current_goal_xy = None   # (px, py) of active goal
        self._arrival_sent = False 
        
        # we don't block here waiting for service; calls will wait or log if not available

        # Asyncio loop & thread for websocket
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._ws_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()

        self.goal_already_sent = False
        # Start background thread running asyncio loop
        self._start_background_loop()
        

        self.get_logger().info(f"SocketListener initialized, target: {self.ws_url}")

    def _start_background_loop(self):
        """asyncio loop in a background daemon thread and websocket main coroutine."""
        self._loop = asyncio.new_event_loop()
        self._ws_thread = threading.Thread(target=self._run_loop, daemon=True)
        self._ws_thread.start()

    def _run_loop(self):
        asyncio.set_event_loop(self._loop)
        try:
            self._loop.run_until_complete(self._ws_main())
        except Exception as e:
            self.get_logger().error(f"Websocket thread exited with exception: {e}")
        finally:
            # cleanup 
            pending = asyncio.all_tasks(loop=self._loop)
            for t in pending:
                t.cancel()
            try:
                self._loop.run_until_complete(asyncio.gather(*pending, return_exceptions=True))
            except Exception:
                pass
            self._loop.close()
    
    ######################################################################
    ######################################################################
    ############################ MAIN WS LOOP ############################
    ######################################################################
    ######################################################################

    async def _ws_main(self):
        """main reconnection loop. On each connection send subscribe_message and process incoming messages."""
        backoff = 1.0  # seconds
        max_backoff = 30.0
        while rclpy.ok() and not self._stop_event.is_set():
            try:
                self.get_logger().info(f"Attempting websocket connect to {self.ws_url}")
                async with websockets.connect(self.ws_url, ping_interval=20, ping_timeout=10) as ws:
                    self.get_logger().info("Websocket connected, sending subscribe messages")
                    try:
                        for msg in self.subscribe_messages:
                            await ws.send(msg)
                    except Exception as e:
                        self.get_logger().warn(f"Failed to send subscribe message: {e}")
                        self._send_to_server("listener", 0, "Error while subscribing to groups")
                        
                        
                    self._send_to_server("listener", 3, "Subscribed to groups")

                    # reset backoff on successful connect
                    backoff = 1.0

                    # listen for messages
                    async for raw_msg in ws:
                        if raw_msg is None:
                            break
                        msg = raw_msg.strip()
                        if not msg:
                            continue
                        self.get_logger().info(f"Websocket received: {msg}")
                        if msg == self.fall_message:
                            self.get_logger().info("FALL received")
                            self._send_to_server("socket", 3, "Fall received")
                            self._send_tts_goal("new fall")
                        if msg == self.stop_message:
                            self.get_logger().info("STOP received")
                            self._send_to_server("socket", 3, "Stop received")
                            self.stop_navigation()
                        elif msg.startswith(self.goto_message):
                            self.get_logger().info("GOTO received")
                            x,y = extract_coordinates(msg)
                            self._send_to_server("socket", 3, f"GOTO received {x=} {y=}")
                            self.send_goal_once(x,y)
                        # elif msg.startswith(self.alrt_message):
                        #     self.get_logger().info("ALERT received")
                        elif msg == self.horn_message:
                            self.get_logger().info("HORN received")
                            self._call_music_play("horn")
                        # ignore all other messages
                    # end of the connection in the websocket
                    self.get_logger().warn("Websocket connection closed, will attempt reconnect")
            except (websockets.ConnectionClosedOK, websockets.ConnectionClosedError) as e:
                self.get_logger().warn(f"Websocket disconnected: {e}")
            except Exception as e:
                self.get_logger().error(f"Websocket connection error: {e}")

            # disconnected -> backoff & retry unless stopping
            if self._stop_event.is_set() or not rclpy.ok():
                break
            self.get_logger().info(f"Reconnect in {backoff:.1f}s...")
            await asyncio.sleep(backoff)
            backoff = min(backoff * 2.0, max_backoff)

        self.get_logger().info("Websocket main coroutine exiting")

    
    ######################################################################
    ######################################################################
    ############################## TTS GOAL ##############################
    ######################################################################
    ######################################################################
    
    def _send_tts_goal(self, text: str):
        """
        Send a TTS goal to the /say action server
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
        
    
    ######################################################################
    ######################################################################
    ############################# MUSIC GOAL #############################
    ######################################################################
    ######################################################################

    def _call_music_play(self, audio_name: str):
        """
        Call the /music_play service asynchronously.
        """
        if not self._music_client.wait_for_service(timeout_sec=1.0):
            # service not available right now
            self.get_logger().warn("/music_play service not available (timeout), skipping play request")
            return

        req = MusicPlay.Request()
        req.audio = audio_name

        # call_async returns a future handled by rclpy executor
        future = self._music_client.call_async(req)

        # callback, log result when ready
        def _on_response(fut):
            try:
                resp = fut.result()
                self.get_logger().info(f"MusicPlay service call finished for '{audio_name}'")
            except Exception as e:
                self.get_logger().error(f"MusicPlay service call failed: {e}")

        future.add_done_callback(lambda fut: _on_response(fut))
        
    
    ######################################################################
    ######################################################################
    ############################# NAV 2 GOAL #############################
    ######################################################################
    ######################################################################

    ############################### START ##############################

    def send_goal_once(self, px, py):
        if self.goal_already_sent:
            self.get_logger().warn('ALREADY IN NAVIGATION (ignoring)')
            self._send_to_server("goto", 1, "Ignoring new goal")
            return

        if not self._client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Nav2 still not ready')
            self._send_to_server("goto", 0, "Nav2 not ready")
            return

        # store goal target and reset arrival flag
        self._current_goal_xy = (px, py)
        self._arrival_sent = False

        self.goal_already_sent = True
        self.get_logger().info(f'Calling nav2 service with coordinates {px=}, {py=}')
        self._send_to_server("goto", 4, f"New destination ({px=}, {py=})...")
        
        msg = Bool()
        msg.data = False
        self.arrived_pub.publish(msg)
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = px
        goal_msg.pose.pose.position.y = py
        goal_msg.pose.pose.orientation.w = 0.3064

        send_goal_future = self._client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)


    def feedback_callback(self, feedback_msg):
        try:
            feedback = feedback_msg.feedback
        except Exception:
            # ignore malformed feedback
            return

        # feedback's current pose if available
        try:
            cur = feedback.current_pose.pose.position
            cur_x = cur.x
            cur_y = cur.y
        except Exception:
            return

        if self._current_goal_xy is None:
            return

        target_x, target_y = self._current_goal_xy
        dist = math.hypot(target_x - cur_x, target_y - cur_y)

        # If within threshold and we haven't handled arrival yet -> handle it
        if dist <= self.arrival_distance and not getattr(self, "_arrival_sent", False):
            self._send_to_server("goto", 2, f"Proximity threshold reached: dist={dist:.3f}m")
            
            self.get_logger().info(f"Proximity threshold reached: dist={dist:.3f} m -> treating as arrived")
            self._arrival_sent = True
            self.goal_already_sent = False
            # publish arrival and try to cancel the active goal to stop Nav2
            self.publish_arrived_flag(True)

            gh = getattr(self, "_current_goal_handle", None)
            if gh is not None:
                try:
                    cancel_future = gh.cancel_goal_async()
                    cancel_future.add_done_callback(lambda f: self.get_logger().info("Cancel request (arrival) completed"))
                    self.get_logger().info("Sent cancel request to Nav2 (arrival)")
                except Exception as e:
                    self.get_logger().error(f"Failed to send cancel on arrival: {e}")
            else:
                self.get_logger().warn("No goal handle to cancel on arrival")


    def result_callback(self, future):
        result = future.result()
        status = result.status

        # STATUS_SUCCEEDED = 4
        if status == 4:
            self.get_logger().info('CAR ARRIVED')
            self._send_to_server("goto", 3, "Car arrived")
            self.publish_arrived_flag(True)
        else:
            self.get_logger().warn(f'GOAL ENDED with status={status}')
            self._send_to_server("goto", 1, f"Goal ended with {status=}")
            self.publish_arrived_flag(False)

    def publish_arrived_flag(self, arrived: bool):
        msg = Bool()
        msg.data = arrived
        self.arrived_pub.publish(msg)
        self._send_to_server("goto", 3, f"Arrival flag {arrived=}")
        
        self.get_logger().info(f'Arrival published on /car_arrived_to_fall = {arrived}')
        self.goal_already_sent = False
        
    ############################### CBCK ###############################
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Rejected goal')
            self._send_to_server("goto", 1, f"Rejected goal")
            self.publish_arrived_flag(False)
            return

        self._send_to_server("goto", 2, f"Goal accepted")
        self.get_logger().info('Goal accepted')
        # store the goal handle so we can cancel later
        self._current_goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    ############################### STOP ###############################
    
    def stop_navigation(self):
        gh = getattr(self, "_current_goal_handle", None)

        if gh is None:
            self._send_to_server("goto", 3, f"No active goal to be stopped")
            self.get_logger().warn("stop_navigation called but no active goal")
            self.goal_already_sent = False
            return

        cancel_future = gh.cancel_goal_async()
        cancel_future.add_done_callback(self._after_cancel)
        self._send_to_server("goto", 3, f"Cancel request sent")
        self.get_logger().info("Sent cancel request to Nav2")

        
    def _after_cancel(self, future):
        try:
            res = future.result()
            self.get_logger().info(f"Cancel response: {res}")
        except Exception as e:
            self.get_logger().error(f"Cancel error: {e}")

        # Reset internal state
        self.goal_already_sent = False
        self._current_goal_handle = None

        # Send a dummy goal to robot's current pose to fully reset BT Navigator
        self.send_goal_to_current_pose()

        
    def send_goal_to_current_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                "map", 
                "base_link",
                rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().error(f"TF lookup failed: {e}")
            return

        px = tf.transform.translation.x
        py = tf.transform.translation.y
        oz = tf.transform.rotation.z
        ow = tf.transform.rotation.w

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = px
        goal_msg.pose.pose.position.y = py
        goal_msg.pose.pose.orientation.z = oz
        goal_msg.pose.pose.orientation.w = ow

        self._send_to_server("goto", 3, f"Go where you are ({px:.3f}, {py:.3f})")

        self.get_logger().info(
            f"Sending dummy goal at current pose ({px:.3f}, {py:.3f})"
        )

        send_future = self._client.send_goal_async(goal_msg)
        send_future.add_done_callback(
            lambda f: self.get_logger().info("Dummy current-pose goal sent")
        )
        
    ######################################################################
    ######################################################################
    ###################### DESTROY OWN SENDER & MAIN #####################
    ######################################################################
    ######################################################################
    
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
        self.get_logger().info("Stopping websocket thread...")
        self._stop_event.set()
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
        node = SocketListener()
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
