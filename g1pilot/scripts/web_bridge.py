#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
WebSocket bridge ROS 2 node.

Listens on a WebSocket and dispatches incoming JSON messages of the shape:

    {
        "id": <int>,
        "function": <str>,
        "request": { ... }
    }

to the corresponding ROS 2 handler. The reply sent back to the client has the
shape:

    {
        "id": <int>,           # echo of the request id
        "function": <str>,     # echo of the requested function
        "status": "ok" | "error",
        "result": { ... },     # present when status == "ok"
        "message": <str>       # present when status == "error"
    }
"""

import asyncio
import json
import signal
import subprocess
import threading
from typing import Any, Awaitable, Callable, Dict, Optional

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

try:
    import websockets
    from websockets.server import WebSocketServerProtocol
except ImportError as exc:
    raise ImportError(
        "The 'websockets' package is required for web_bridge. "
        "Install it with: pip install websockets"
    ) from exc


HandlerFn = Callable[[Dict[str, Any]], Awaitable[Dict[str, Any]]]


class WebBridge(Node):
    def __init__(self) -> None:
        super().__init__('web_bridge')

        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8765)
        self.declare_parameter(
            'motion_sequence_binary',
            '/sdk2/unitree_sdk2/build/bin/g1_arm_replay_no_rewind',
        )
        self.declare_parameter('motion_sequence_cwd', '/sdk2/unitree_sdk2')
        self.declare_parameter('motion_sequence_interface', 'eno2')
        self.declare_parameter('motion_sequence_file', 'data/tray1.seq')

        self.host: str = self.get_parameter('host').get_parameter_value().string_value
        self.port: int = int(self.get_parameter('port').value)

        self.goal_pub = self.create_publisher(PoseStamped, '/g1pilot/goal', 10)
        self.create_subscription(Bool, '/g1pilot/goal_reach', self._on_goal_reach, 10)

        self._motion_proc: Optional[subprocess.Popen] = None
        self._motion_lock = threading.Lock()
        self._pending_lock = threading.Lock()
        self._pending_nav: Dict[Any, int] = {}
        self._loop: Optional[asyncio.AbstractEventLoop] = None

        self._handlers: Dict[str, HandlerFn] = {
            'start_navigation': self.handle_start_navigation,
            'start_motion_sequence': self.handle_start_motion_sequence,
            'stop_motion_sequence': self.handle_stop_motion_sequence,
        }

        self.get_logger().info(
            f"WebBridge ready. Registered functions: {list(self._handlers)}"
        )

    def register_pending_navigation(
        self, ws: 'WebSocketServerProtocol', msg_id: Any
    ) -> None:
        with self._pending_lock:
            self._pending_nav[ws] = msg_id

    def unregister_client(self, ws: 'WebSocketServerProtocol') -> None:
        with self._pending_lock:
            self._pending_nav.pop(ws, None)

    def _on_goal_reach(self, msg: Bool) -> None:
        if not msg.data or self._loop is None:
            return
        self.get_logger().info('Goal reached — notifying WebSocket client(s).')
        asyncio.run_coroutine_threadsafe(self._notify_goal_reached_all(), self._loop)

    async def _notify_goal_reached_all(self) -> None:
        with self._pending_lock:
            pending = list(self._pending_nav.items())
            self._pending_nav.clear()

        for ws, msg_id in pending:
            response = {
                'id': msg_id,
                'function': 'start_navigation',
                'status': 'ok',
                'result': {'message': 'goal reached'},
            }
            try:
                await ws.send(json.dumps(response))
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"Failed to notify client of goal reach: {exc}")

    # ------------------------------------------------------------------ #
    # ROS 2 handlers (stubs)                                             #
    # ------------------------------------------------------------------ #
    async def handle_start_navigation(self, request: Dict[str, Any]) -> Dict[str, Any]:
        """
        Publish a PoseStamped goal on `/g1pilot/goal`.

        Equivalent to:
            ros2 topic pub --once /g1pilot/goal geometry_msgs/PoseStamped \
                "{header: {frame_id: 'map'},
                  pose: {position: {x: 0.0, y: 0.0, z: 0.0},
                         orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"

        `request` may override any of: `frame_id`, `position.{x,y,z}`,
        `orientation.{x,y,z,w}`.
        """
        self.get_logger().info(f"start_navigation called with: {request}")

        position = request.get('position', {}) or {}
        orientation = request.get('orientation', {}) or {}

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(request.get('frame_id', 'map'))
        msg.pose.position.x = float(position.get('x', 0.0))
        msg.pose.position.y = float(position.get('y', 0.0))
        msg.pose.position.z = float(position.get('z', 0.0))
        msg.pose.orientation.x = float(orientation.get('x', 0.0))
        msg.pose.orientation.y = float(orientation.get('y', 0.0))
        msg.pose.orientation.z = float(orientation.get('z', 0.0))
        msg.pose.orientation.w = float(orientation.get('w', 1.0))

        self.goal_pub.publish(msg)
        self.get_logger().info(
            f"Published goal on /g1pilot/goal: frame={msg.header.frame_id}, "
            f"pos=({msg.pose.position.x}, {msg.pose.position.y}, {msg.pose.position.z})"
        )

        return {
            'topic': '/g1pilot/goal',
            'frame_id': msg.header.frame_id,
            'position': {
                'x': msg.pose.position.x,
                'y': msg.pose.position.y,
                'z': msg.pose.position.z,
            },
            'orientation': {
                'x': msg.pose.orientation.x,
                'y': msg.pose.orientation.y,
                'z': msg.pose.orientation.z,
                'w': msg.pose.orientation.w,
            },
        }

    async def handle_start_motion_sequence(self, request: Dict[str, Any]) -> Dict[str, Any]:
        """
        Spawn the g1_arm_replay_no_rewind binary as a subprocess.

        The binary plays a recorded .seq file through rt/arm_sdk and then
        holds the final pose until ENTER or SIGUSR1 (see handle_stop_motion_sequence).

        `request` may override:
            binary:    absolute path to the executable
            cwd:       working directory (resolves the relative seq file)
            interface: network interface arg (e.g. 'eno2')
            file:      seq file path passed to the binary
        """
        self.get_logger().info(f"start_motion_sequence called with: {request}")

        binary = str(request.get(
            'binary',
            self.get_parameter('motion_sequence_binary').get_parameter_value().string_value,
        ))
        cwd = str(request.get(
            'cwd',
            self.get_parameter('motion_sequence_cwd').get_parameter_value().string_value,
        ))
        interface = str(request.get(
            'interface',
            self.get_parameter('motion_sequence_interface').get_parameter_value().string_value,
        ))
        seq_file = str(request.get(
            'file',
            self.get_parameter('motion_sequence_file').get_parameter_value().string_value,
        ))

        with self._motion_lock:
            if self._motion_proc is not None and self._motion_proc.poll() is None:
                raise RuntimeError(
                    f"Motion sequence already running (pid={self._motion_proc.pid})"
                )

            cmd = [binary, interface, seq_file]
            self.get_logger().info(f"Launching motion sequence: {cmd} (cwd={cwd})")
            proc = subprocess.Popen(
                cmd,
                cwd=cwd,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
            )
            self._motion_proc = proc

        return {
            'pid': proc.pid,
            'binary': binary,
            'interface': interface,
            'file': seq_file,
            'cwd': cwd,
        }

    async def handle_stop_motion_sequence(self, request: Dict[str, Any]) -> Dict[str, Any]:
        """
        Signal a running motion sequence to release (SIGUSR1).

        The binary handles SIGUSR1 by exiting the hold phase and ramping
        arm control down cleanly.
        """
        self.get_logger().info(f"stop_motion_sequence called with: {request}")

        with self._motion_lock:
            proc = self._motion_proc
            if proc is None:
                raise RuntimeError("No motion sequence is running")
            if proc.poll() is not None:
                self._motion_proc = None
                raise RuntimeError(
                    f"Motion sequence already exited (code={proc.returncode})"
                )

            proc.send_signal(signal.SIGUSR1)
            pid = proc.pid

        return {
            'pid': pid,
            'signal': 'SIGUSR1',
        }

    # ------------------------------------------------------------------ #
    # Dispatch                                                           #
    # ------------------------------------------------------------------ #
    async def dispatch(self, message: Dict[str, Any]) -> Dict[str, Any]:
        msg_id = message.get('id')
        function = message.get('function')
        request = message.get('request', {}) or {}

        if not isinstance(function, str):
            return {
                'id': msg_id,
                'function': function,
                'status': 'error',
                'message': "Missing or invalid 'function' field",
            }

        handler = self._handlers.get(function)
        if handler is None:
            return {
                'id': msg_id,
                'function': function,
                'status': 'error',
                'message': f"Unknown function '{function}'",
            }

        if not isinstance(request, dict):
            return {
                'id': msg_id,
                'function': function,
                'status': 'error',
                'message': "'request' must be an object",
            }

        try:
            result = await handler(request)
        except Exception as exc:  # noqa: BLE001 - report any handler failure to client
            self.get_logger().error(f"Handler '{function}' raised: {exc}")
            return {
                'id': msg_id,
                'function': function,
                'status': 'error',
                'message': str(exc),
            }

        return {
            'id': msg_id,
            'function': function,
            'status': 'ok',
            'result': result if isinstance(result, dict) else {},
        }


# ---------------------------------------------------------------------- #
# WebSocket server                                                       #
# ---------------------------------------------------------------------- #
async def _serve(node: WebBridge) -> None:
    node._loop = asyncio.get_running_loop()

    async def handler(ws: 'WebSocketServerProtocol') -> None:
        peer = getattr(ws, 'remote_address', None)
        node.get_logger().info(f"Client connected: {peer}")
        try:
            async for raw in ws:
                try:
                    message = json.loads(raw)
                except json.JSONDecodeError as exc:
                    await ws.send(json.dumps({
                        'id': None,
                        'function': None,
                        'status': 'error',
                        'message': f"Invalid JSON: {exc}",
                    }))
                    continue

                if not isinstance(message, dict):
                    await ws.send(json.dumps({
                        'id': None,
                        'function': None,
                        'status': 'error',
                        'message': 'Payload must be a JSON object',
                    }))
                    continue

                response = await node.dispatch(message)
                await ws.send(json.dumps(response))
                if (
                    message.get('function') == 'start_navigation'
                    and response.get('status') == 'ok'
                ):
                    node.register_pending_navigation(ws, message.get('id'))
        except websockets.ConnectionClosed:
            pass
        finally:
            node.unregister_client(ws)
            node.get_logger().info(f"Client disconnected: {peer}")

    node.get_logger().info(f"WebSocket server listening on ws://{node.host}:{node.port}")
    async with websockets.serve(handler, node.host, node.port):
        await asyncio.Future()  # run forever


def _spin_ros(executor: SingleThreadedExecutor, stop_event: threading.Event) -> None:
    while not stop_event.is_set() and rclpy.ok():
        executor.spin_once(timeout_sec=0.1)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WebBridge()

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    stop_event = threading.Event()
    ros_thread = threading.Thread(
        target=_spin_ros, args=(executor, stop_event), daemon=True
    )
    ros_thread.start()

    try:
        asyncio.run(_serve(node))
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        ros_thread.join(timeout=1.0)
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
