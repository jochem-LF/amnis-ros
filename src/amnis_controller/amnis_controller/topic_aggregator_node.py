#!/usr/bin/env python3
"""Topic aggregation node that exposes ROS 2 data over WebSockets.

This node discovers (most) topics available in the ROS graph, subscribes to
them dynamically, converts each incoming message to JSON-friendly data, keeps a
cache of the most recent sample per topic, and streams updates to connected
WebSocket clients.  The goal is to provide a lightweight data layer that a
separate frontend can consume without speaking ROS 2 directly.

Limitations:
    * Topics that use custom or un-importable message types are skipped.
    * Binary-heavy messages (e.g. images) are base64-encoded, which may be
      expensive for high-volume streams.
    * The node only ever keeps the most recent sample per topic and does not
      replay history beyond a single snapshot on connect.

Despite those caveats, this is a practical starting point for prototyping a
browser-based dashboard or inspector for ROS 2 systems.
"""

from __future__ import annotations

import asyncio
import base64
import json
import signal
import threading
from array import array
from copy import deepcopy
from dataclasses import dataclass
from typing import Any, Dict, Iterable, Optional, Sequence, Set

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.subscription import Subscription
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message

try:
    import websockets
    from websockets.server import WebSocketServerProtocol, WebSocketServer
except ModuleNotFoundError as exc:  # pragma: no cover - handled at runtime
    raise RuntimeError(
        'The "websockets" Python package is required for the topic_aggregator_node.'
    ) from exc


@dataclass(frozen=True)
class TopicSample:
    """Container for the latest sample of a topic."""

    topic: str
    type_name: str
    data: Dict[str, Any]
    received_stamp: Dict[str, int]
    header_stamp: Optional[Dict[str, int]]


class TopicAggregatorNode(Node):
    """ROS 2 node that bridges topic data to WebSocket clients."""

    def __init__(self) -> None:
        super().__init__('topic_aggregator')

        # Parameters -----------------------------------------------------
        self.declare_parameter('topic_poll_interval', 2.0)
        self.declare_parameter('include_hidden_topics', False)
        self.declare_parameter('ignored_topics', ['/parameter_events', '/rosout'])
        self.declare_parameter('websocket_host', '0.0.0.0')
        self.declare_parameter('websocket_port', 8765)
        self.declare_parameter('update_frequency_hz', 0.5)

        poll_interval = float(self.get_parameter('topic_poll_interval').value)
        self._include_hidden_topics = bool(
            self.get_parameter('include_hidden_topics').value
        )

        ignored_param = self.get_parameter('ignored_topics').value
        self._ignored_topics: Set[str] = self._normalize_iterable_param(ignored_param)

        self._websocket_host = str(self.get_parameter('websocket_host').value)
        self._websocket_port = int(self.get_parameter('websocket_port').value)
        
        update_freq_hz = float(self.get_parameter('update_frequency_hz').value)
        self._update_period_sec = max(0.1, 1.0 / max(update_freq_hz, 0.01))  # Clamp to reasonable range

        # ROS state ------------------------------------------------------
        self._topic_subscriptions: Dict[str, Subscription] = {}
        self._topic_types: Dict[str, str] = {}
        self._latest_samples: Dict[str, TopicSample] = {}
        self._latest_lock = threading.Lock()

        # WebSocket infrastructure --------------------------------------
        self._websocket_loop = asyncio.new_event_loop()
        self._websocket_thread = threading.Thread(
            target=self._run_websocket_thread,
            name='topic-aggregator-ws',
            daemon=True,
        )
        self._ws_ready = threading.Event()
        self._ws_shutdown_requested = threading.Event()
        self._ws_startup_error: Optional[str] = None

        # These attributes are initialised inside the WebSocket thread but
        # declared here for type-checkers and clarity.
        self._broadcast_queue: Optional[asyncio.Queue[str]] = None
        self._shutdown_asyncio_event: Optional[asyncio.Event] = None
        self._ws_clients: Optional[Set[WebSocketServerProtocol]] = None
        self._ws_server: Optional[WebSocketServer] = None

        self._websocket_thread.start()
        if not self._ws_ready.wait(timeout=5.0):
            self.get_logger().warning('WebSocket server did not signal readiness within 5 seconds.')
        if self._ws_startup_error:
            self.get_logger().error(
                f'WebSocket server failed to start: {self._ws_startup_error}'
            )

        # Start topic discovery timer and do an initial discovery pass.
        self._topic_poll_timer = self.create_timer(  # type: ignore[assignment]
            max(poll_interval, 0.5),
            self._discover_topics,
        )
        self._discover_topics()
        
        # Start periodic update timer for throttled broadcasts
        self._update_timer = self.create_timer(  # type: ignore[assignment]
            self._update_period_sec,
            self._send_periodic_updates,
        )

        self.get_logger().info(
            'Topic aggregator node ready. Listening for topics and serving WebSocket '
            f'clients on ws://{self._websocket_host}:{self._websocket_port} '
            f'(update frequency: {update_freq_hz:.2f}Hz)'
        )

    # ------------------------------------------------------------------
    # WebSocket lifecycle
    # ------------------------------------------------------------------
    def _run_websocket_thread(self) -> None:
        asyncio.set_event_loop(self._websocket_loop)

        async def main() -> None:
            self._broadcast_queue = asyncio.Queue()
            self._shutdown_asyncio_event = asyncio.Event()
            self._ws_clients = set()
            try:
                # Create WebSocket server
                # Note: SO_REUSEADDR is enabled by default in most asyncio servers
                self._ws_server = await websockets.serve(
                    self._handle_client,
                    self._websocket_host,
                    self._websocket_port,
                    ping_interval=20.0,
                    ping_timeout=20.0,
                )
                self._ws_ready.set()
                self.get_logger().info(
                    f'WebSocket server started on ws://{self._websocket_host}:{self._websocket_port}'
                )
                await self._broadcast_consumer()
            except Exception as exc:  # pragma: no cover - runtime safety
                self._ws_startup_error = str(exc)
                self._ws_ready.set()
                self.get_logger().error(f'WebSocket server terminated: {exc}')
            finally:
                # Properly close the server
                if self._ws_server is not None:
                    self.get_logger().info('Closing WebSocket server...')
                    self._ws_server.close()
                    await self._ws_server.wait_closed()
                    self.get_logger().info('WebSocket server closed.')

        try:
            self._websocket_loop.run_until_complete(main())
        finally:
            pending = asyncio.all_tasks(self._websocket_loop)
            for task in pending:
                task.cancel()
            if pending:
                self._websocket_loop.run_until_complete(
                    asyncio.gather(*pending, return_exceptions=True)
                )
            self._websocket_loop.close()

    async def _broadcast_consumer(self) -> None:
        assert self._broadcast_queue is not None
        assert self._shutdown_asyncio_event is not None
        while not self._ws_shutdown_requested.is_set():
            try:
                payload = await asyncio.wait_for(self._broadcast_queue.get(), timeout=0.5)
            except asyncio.TimeoutError:
                if self._shutdown_asyncio_event.is_set():
                    break
                continue
            await self._broadcast_to_clients(payload)
        
        # Gracefully close all connected clients on shutdown
        if self._ws_clients:
            self.get_logger().info(f'Closing {len(self._ws_clients)} WebSocket client(s)...')
            close_tasks = [
                client.close(code=1001, reason='Server shutdown') 
                for client in list(self._ws_clients)
            ]
            await asyncio.gather(*close_tasks, return_exceptions=True)
            # Wait a bit for clients to properly close
            await asyncio.sleep(0.1)
            self._ws_clients.clear()

    async def _broadcast_to_clients(self, payload: str) -> None:
        if not self._ws_clients:
            return
        disconnected: Set[WebSocketServerProtocol] = set()
        for client in list(self._ws_clients):
            try:
                await client.send(payload)
            except websockets.ConnectionClosed:
                disconnected.add(client)
            except Exception as exc:  # pragma: no cover - best-effort logging
                self.get_logger().warning(f'Failed to broadcast to client: {exc}')
                disconnected.add(client)
        for client in disconnected:
            self._ws_clients.discard(client)

    async def _handle_client(self, websocket: WebSocketServerProtocol) -> None:
        assert self._ws_clients is not None
        self._ws_clients.add(websocket)
        try:
            snapshot_payload = self._build_snapshot_payload()
            await websocket.send(snapshot_payload)
            # Keep the connection open until the client disconnects.
            await websocket.wait_closed()
        finally:
            self._ws_clients.discard(websocket)

    def _enqueue_payload(self, payload: str) -> None:
        if not self._ws_ready.is_set() or not self._broadcast_queue:
            return
        try:
            asyncio.run_coroutine_threadsafe(
                self._broadcast_queue.put(payload),
                self._websocket_loop,
            )
        except RuntimeError:
            # Loop is probably shutting down.
            pass

    # ------------------------------------------------------------------
    # Topic discovery and message handling
    # ------------------------------------------------------------------
    def _send_periodic_updates(self) -> None:
        """Send batched updates for all topics at the configured frequency."""
        with self._latest_lock:
            if not self._latest_samples:
                return
            # Create a batch update with all current samples
            updates = {
                topic: {
                    'topic': sample.topic,
                    'type': sample.type_name,
                    'data': deepcopy(sample.data),
                    'received_stamp': deepcopy(sample.received_stamp),
                    'header_stamp': deepcopy(sample.header_stamp),
                }
                for topic, sample in self._latest_samples.items()
            }
        
        payload = json.dumps(
            {
                'event': 'batch_update',
                'payload': {
                    'topics': updates,
                    'count': len(updates),
                },
            },
            default=self._json_default,
        )
        self._enqueue_payload(payload)
    
    def _discover_topics(self) -> None:
        try:
            topics_and_types = self.get_topic_names_and_types(
                include_hidden_topics=self._include_hidden_topics
            )
        except TypeError:
            if self._include_hidden_topics:
                self.get_logger().warning(
                    'include_hidden_topics parameter not supported by this ROS 2 distribution; '
                    'continuing with default visibility.'
                )
            try:
                topics_and_types = self.get_topic_names_and_types()
            except Exception as exc:  # pragma: no cover - defensive logging
                self.get_logger().warning(f'Failed to fetch topic names and types: {exc}')
                return
        except Exception as exc:  # pragma: no cover - defensive logging
            self.get_logger().warning(f'Failed to fetch topic names and types: {exc}')
            return

        for topic_name, type_list in topics_and_types:
            if topic_name in self._ignored_topics:
                continue
            if topic_name in self._topic_subscriptions:
                continue
            if not type_list:
                continue
            type_name = type_list[0]
            if len(type_list) > 1:
                self.get_logger().warning(
                    f'Topic {topic_name} has multiple types {type_list}; using {type_name}. '
                    'Mixed-type publishers are not fully supported.'
                )
            try:
                msg_type = get_message(type_name)
            except (ModuleNotFoundError, ValueError) as exc:
                self.get_logger().warning(
                    f'Unable to import message type "{type_name}" for topic {topic_name}: {exc}'
                )
                continue

            subscription = self.create_subscription(
                msg_type,
                topic_name,
                self._build_topic_callback(topic_name, type_name),
                QoSProfile(depth=10),
            )
            self._topic_subscriptions[topic_name] = subscription
            self._topic_types[topic_name] = type_name

            topic_added_payload = json.dumps(
                {
                    'event': 'topic_added',
                    'payload': {
                        'topic': topic_name,
                        'type': type_name,
                    },
                }
            )
            self._enqueue_payload(topic_added_payload)

    def _build_topic_callback(self, topic_name: str, type_name: str):
        def _callback(msg: Any) -> None:
            converted = self._ros_message_to_jsonable(msg)
            received_stamp_msg = self.get_clock().now().to_msg()
            received_stamp = {
                'sec': int(received_stamp_msg.sec),
                'nanosec': int(received_stamp_msg.nanosec),
            }

            header_stamp: Optional[Dict[str, int]] = None
            if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
                try:
                    header_stamp = {
                        'sec': int(msg.header.stamp.sec),
                        'nanosec': int(msg.header.stamp.nanosec),
                    }
                except AttributeError:
                    header_stamp = None

            sample = TopicSample(
                topic=topic_name,
                type_name=type_name,
                data=converted,
                received_stamp=received_stamp,
                header_stamp=header_stamp,
            )

            with self._latest_lock:
                self._latest_samples[topic_name] = sample

            # Don't enqueue immediately - updates will be sent periodically by timer
            # This reduces resource usage by batching updates

        return _callback

    # ------------------------------------------------------------------
    # Utilities
    # ------------------------------------------------------------------
    def _json_default(self, value: Any) -> Any:
        if isinstance(value, (bytes, bytearray)):
            return base64.b64encode(value).decode('ascii')
        if isinstance(value, array):
            return list(value)
        return str(value)

    def _ros_message_to_jsonable(self, msg: Any) -> Dict[str, Any]:
        try:
            ordered = message_to_ordereddict(msg)
        except Exception as exc:  # pragma: no cover - fallback path
            self.get_logger().warning(f'Failed to convert message on {type(msg)}: {exc}')
            return {'__raw__': str(msg)}
        return self._sanitize_for_json(ordered)

    def _sanitize_for_json(self, value: Any) -> Any:
        if isinstance(value, (str, int, float, bool)) or value is None:
            return value
        if isinstance(value, dict):
            return {str(k): self._sanitize_for_json(v) for k, v in value.items()}
        if isinstance(value, (list, tuple)):
            return [self._sanitize_for_json(item) for item in value]
        if isinstance(value, array):
            return [self._sanitize_for_json(item) for item in value.tolist()]
        if isinstance(value, (bytes, bytearray, memoryview)):
            return base64.b64encode(bytes(value)).decode('ascii')
        return str(value)

    def _build_snapshot_payload(self) -> str:
        with self._latest_lock:
            snapshot = {
                topic: {
                    'topic': sample.topic,
                    'type': sample.type_name,
                    'data': deepcopy(sample.data),
                    'received_stamp': deepcopy(sample.received_stamp),
                    'header_stamp': deepcopy(sample.header_stamp),
                }
                for topic, sample in self._latest_samples.items()
            }
        return json.dumps(
            {'event': 'snapshot', 'payload': {'topics': snapshot, 'count': len(snapshot)}},
            default=self._json_default,
        )

    def _normalize_iterable_param(self, param_value: Any) -> Set[str]:
        if isinstance(param_value, (list, tuple, set)):
            iterable: Iterable[Any] = param_value
        elif param_value is None:
            iterable = []
        else:
            iterable = [param_value]
        return {str(item) for item in iterable if str(item)}

    # ------------------------------------------------------------------
    # Shutdown & cleanup
    # ------------------------------------------------------------------
    def destroy_node(self) -> bool:
        self._request_websocket_shutdown()

        for topic, subscription in list(self._topic_subscriptions.items()):
            try:
                self.destroy_subscription(subscription)
            except Exception:
                pass
            finally:
                self._topic_subscriptions.pop(topic, None)

        return super().destroy_node()

    def _request_websocket_shutdown(self) -> None:
        if self._ws_shutdown_requested.is_set():
            return
        self._ws_shutdown_requested.set()
        if self._shutdown_asyncio_event is not None:
            self._websocket_loop.call_soon_threadsafe(self._shutdown_asyncio_event.set)
        if self._websocket_thread.is_alive():
            self._websocket_thread.join(timeout=5.0)


def main(args: Sequence[str] | None = None) -> None:
    """Entry point for the topic aggregator node."""

    rclpy.init(args=args)
    node = TopicAggregatorNode()
    
    # Set up signal handlers for graceful shutdown
    def signal_handler(signum, frame):
        node.get_logger().info(f'Received signal {signum}, initiating shutdown...')
        raise KeyboardInterrupt
    
    # Register handlers for SIGINT and SIGTERM
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:  # pragma: no cover - user interruption
        node.get_logger().info('Shutting down gracefully...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


__all__ = ['TopicAggregatorNode']


if __name__ == '__main__':
    main()


