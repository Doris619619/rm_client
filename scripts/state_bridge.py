#!/usr/bin/env python3

import copy
import base64
import json
import threading
import time
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Dict

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, UInt8MultiArray


CUSTOM_MAGIC_0 = 0x52
CUSTOM_MAGIC_1 = 0x4D
CUSTOM_VERSION = 0x01
CUSTOM_MSG_TYPE_IMAGE = 0x01
CUSTOM_ENCODING_GRAY8 = 0x01
CUSTOM_IMAGE_WIDTH = 160
CUSTOM_IMAGE_HEIGHT = 120
CUSTOM_FRAME_TIMEOUT_SEC = 1.0


def now_iso_utc() -> str:
    return datetime.now(timezone.utc).isoformat()


class SharedState:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._state: Dict[str, Any] = {
            "game_status": None,
            "global_unit_status": None,
            "robot_dynamic_status": None,
            "guard_ctrl_result": None,
            "custom_image": None,
            "meta": {
                "last_update_game_status": None,
                "last_update_global_unit_status": None,
                "last_update_robot_dynamic_status": None,
                "last_update_guard_ctrl_result": None,
                "bridge_started_at": now_iso_utc(),
                "bridge_alive": True,
            },
        }
        self._first_seen: Dict[str, bool] = {
            "game_status": False,
            "global_unit_status": False,
            "robot_dynamic_status": False,
            "guard_ctrl_result": False,
        }
        self._custom_image_first_seen = False
        self._pending_custom_frames: Dict[int, Dict[str, Any]] = {}

    def _cleanup_custom_frames_locked(self, now_mono: float) -> None:
        expired = [
            frame_id
            for frame_id, frame in self._pending_custom_frames.items()
            if now_mono - frame["created_at_mono"] > CUSTOM_FRAME_TIMEOUT_SEC
        ]
        for frame_id in expired:
            del self._pending_custom_frames[frame_id]

    def update_custom_image_chunk(self, data: bytes) -> bool:
        if len(data) < 12:
            return False
        if data[0] != CUSTOM_MAGIC_0 or data[1] != CUSTOM_MAGIC_1:
            return False
        if data[2] != CUSTOM_VERSION or data[3] != CUSTOM_MSG_TYPE_IMAGE:
            return False

        frame_id = data[4] | (data[5] << 8)
        chunk_id = data[6]
        total_chunks = data[7]
        width = data[8]
        height = data[9]
        encoding = data[10]

        if total_chunks == 0 or chunk_id >= total_chunks:
            return False
        if width != CUSTOM_IMAGE_WIDTH or height != CUSTOM_IMAGE_HEIGHT:
            return False
        if encoding != CUSTOM_ENCODING_GRAY8:
            return False

        payload = bytes(data[12:])
        now_mono = time.monotonic()
        expected_size = CUSTOM_IMAGE_WIDTH * CUSTOM_IMAGE_HEIGHT

        with self._lock:
            self._cleanup_custom_frames_locked(now_mono)

            frame = self._pending_custom_frames.get(frame_id)
            if frame is None:
                frame = {
                    "created_at_mono": now_mono,
                    "total_chunks": total_chunks,
                    "width": width,
                    "height": height,
                    "encoding": encoding,
                    "chunks": {},
                }
                self._pending_custom_frames[frame_id] = frame
            else:
                if (
                    frame["total_chunks"] != total_chunks
                    or frame["width"] != width
                    or frame["height"] != height
                    or frame["encoding"] != encoding
                ):
                    return False

            frame["chunks"][chunk_id] = payload

            if len(frame["chunks"]) != total_chunks:
                return False

            try:
                merged = b"".join(frame["chunks"][i] for i in range(total_chunks))
            except KeyError:
                return False

            if len(merged) < expected_size:
                del self._pending_custom_frames[frame_id]
                return False

            image_bytes = merged[:expected_size]
            self._state["custom_image"] = {
                "frame_id": frame_id,
                "width": CUSTOM_IMAGE_WIDTH,
                "height": CUSTOM_IMAGE_HEIGHT,
                "encoding": "GRAY8",
                "updated_at": now_iso_utc(),
                "data_base64": base64.b64encode(image_bytes).decode("ascii"),
            }
            del self._pending_custom_frames[frame_id]

            first = not self._custom_image_first_seen
            if first:
                self._custom_image_first_seen = True
            return first

    def update(self, key: str, raw_text: str, meta_time_field: str) -> bool:
        try:
            parsed = json.loads(raw_text) if raw_text else None
        except json.JSONDecodeError:
            parsed = {"raw": raw_text}

        with self._lock:
            self._state[key] = parsed
            self._state["meta"][meta_time_field] = now_iso_utc()
            first_message = not self._first_seen[key]
            if first_message:
                self._first_seen[key] = True
            return first_message

    def set_bridge_alive(self, alive: bool) -> None:
        with self._lock:
            self._state["meta"]["bridge_alive"] = alive

    def snapshot(self) -> Dict[str, Any]:
        with self._lock:
            self._cleanup_custom_frames_locked(time.monotonic())
            return copy.deepcopy(self._state)


class StateBridgeNode(Node):
    def __init__(self, shared_state: SharedState) -> None:
        super().__init__("state_bridge")
        self._shared_state = shared_state

        self._topic_defs = {
            "/judge/game_status": ("game_status", "last_update_game_status"),
            "/judge/global_unit_status": ("global_unit_status", "last_update_global_unit_status"),
            "/judge/robot_dynamic_status": (
                "robot_dynamic_status",
                "last_update_robot_dynamic_status",
            ),
            "/judge/guard_ctrl_result": ("guard_ctrl_result", "last_update_guard_ctrl_result"),
        }

        self._subs = []
        for topic_name, (state_key, meta_time_field) in self._topic_defs.items():
            sub = self.create_subscription(
                String,
                topic_name,
                self._make_callback(topic_name, state_key, meta_time_field),
                10,
            )
            self._subs.append(sub)

        self._custom_sub = self.create_subscription(
            UInt8MultiArray,
            "/judge/custom_byte_block",
            self._on_custom_block,
            200,
        )

    def _make_callback(self, topic_name: str, state_key: str, meta_time_field: str):
        def _on_msg(msg: String) -> None:
            first = self._shared_state.update(state_key, msg.data, meta_time_field)
            if first:
                self.get_logger().info(f"收到首条消息: {topic_name}")

        return _on_msg

    def _on_custom_block(self, msg: UInt8MultiArray) -> None:
        first = self._shared_state.update_custom_image_chunk(bytes(msg.data))
        if first:
            self.get_logger().info("收到首帧辅助图像")


class StateRequestHandler(BaseHTTPRequestHandler):
    shared_state: SharedState = None

    def _send_cors_headers(self) -> None:
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")

    def do_OPTIONS(self) -> None:
        self.send_response(204)
        self._send_cors_headers()
        self.end_headers()

    def do_GET(self) -> None:
        if self.path == "/api/state":
            body = json.dumps(self.shared_state.snapshot(), ensure_ascii=False).encode("utf-8")
            self.send_response(200)
            self._send_cors_headers()
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        body = json.dumps({"error": "not_found"}).encode("utf-8")
        self.send_response(404)
        self._send_cors_headers()
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, format: str, *args) -> None:
        return


def start_http_server(shared_state: SharedState, host: str = "127.0.0.1", port: int = 8000):
    StateRequestHandler.shared_state = shared_state
    server = ThreadingHTTPServer((host, port), StateRequestHandler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server


def main() -> None:
    rclpy.init()
    shared_state = SharedState()
    node = StateBridgeNode(shared_state)

    node.get_logger().info("bridge 启动成功")
    http_server = start_http_server(shared_state, host="127.0.0.1", port=8000)
    node.get_logger().info("HTTP 服务启动成功: http://127.0.0.1:8000/api/state")

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        shared_state.set_bridge_alive(False)
        http_server.shutdown()
        http_server.server_close()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
