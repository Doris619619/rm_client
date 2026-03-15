#!/usr/bin/env python3

import copy
import json
import threading
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Dict

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


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

    def _make_callback(self, topic_name: str, state_key: str, meta_time_field: str):
        def _on_msg(msg: String) -> None:
            first = self._shared_state.update(state_key, msg.data, meta_time_field)
            if first:
                self.get_logger().info(f"收到首条消息: {topic_name}")

        return _on_msg


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
    except KeyboardInterrupt:
        pass
    finally:
        shared_state.set_bridge_alive(False)
        http_server.shutdown()
        http_server.server_close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
