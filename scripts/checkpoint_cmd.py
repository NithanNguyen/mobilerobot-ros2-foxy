#!/usr/bin/env python3
"""
checkpoint_cmd_2.py — Clean Terminal Edition
=============================================
Interactive CLI for navigator_v2.py.

Only 4 user commands are accepted:
    go <id>
    stop
    continue
    reset

ROS interface:
    Pub: /robot/command              std_msgs/String
         "go:<id>", "stop", "continue", "reset"
    Sub: /robot/state                std_msgs/String
    Sub: /robot/current_checkpoint   std_msgs/Int32
    Sub: /robot/status_message       std_msgs/String

Exit:
    Ctrl+C or EOF.
"""

import os
import sys
import time
import threading
from typing import Dict, Optional

import yaml
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String, Int32


# ============================================================
#  Display policy
# ============================================================
# True  : dùng màu ANSI cho terminal dễ nhìn.
# False : không dùng màu, phù hợp khi ghi log ra file.
USE_COLOR = True

# Không in lặp lại cùng một status từ /robot/status_message.
PRINT_STATUS_MESSAGE = True

# Nếu navigator_v2.py chưa chạy, chỉ cảnh báo 1 lần để tránh spam.
WARN_NO_NAVIGATOR_ONCE = True


# ============================================================
#  Terminal color helper
# ============================================================
class C:
    RESET = "\033[0m" if USE_COLOR else ""
    BOLD = "\033[1m" if USE_COLOR else ""
    DIM = "\033[2m" if USE_COLOR else ""
    RED = "\033[91m" if USE_COLOR else ""
    GREEN = "\033[92m" if USE_COLOR else ""
    YELLOW = "\033[93m" if USE_COLOR else ""
    CYAN = "\033[96m" if USE_COLOR else ""

    @staticmethod
    def bold(s: str) -> str:
        return f"{C.BOLD}{s}{C.RESET}"

    @staticmethod
    def dim(s: str) -> str:
        return f"{C.DIM}{s}{C.RESET}"

    @staticmethod
    def ok(s: str) -> str:
        return f"{C.GREEN}{s}{C.RESET}"

    @staticmethod
    def warn(s: str) -> str:
        return f"{C.YELLOW}{s}{C.RESET}"

    @staticmethod
    def err(s: str) -> str:
        return f"{C.RED}{s}{C.RESET}"

    @staticmethod
    def info(s: str) -> str:
        return f"{C.CYAN}{s}{C.RESET}"


STATE_HINTS = {
    "unknown": "wait",
    "IDLE": "go <id>",
    "COMPUTING_PATH": "stop",
    "PRE_ROTATING": "stop",
    "NAVIGATING": "stop",
    "STOPPED": "continue | reset",
    "WAITING_RESET": "go <id> | wait home",
    "RETURNING_HOME": "wait",
}

STATE_COLOR = {
    "IDLE": C.ok,
    "COMPUTING_PATH": C.info,
    "PRE_ROTATING": C.info,
    "NAVIGATING": C.info,
    "STOPPED": C.warn,
    "WAITING_RESET": C.warn,
    "RETURNING_HOME": C.info,
    "unknown": C.dim,
}

CHECKPOINTS: Dict[int, str] = {}


# ============================================================
#  Checkpoint loader
# ============================================================
def load_checkpoints() -> str:
    """Load config/checkpoints_v2.yaml from installed mobile_robot package."""
    try:
        pkg = get_package_share_directory("mobile_robot")
        path = os.path.join(pkg, "config", "checkpoints_e6.yaml")

        with open(path, "r") as f:
            data = yaml.safe_load(f) or {}

        CHECKPOINTS.clear()
        for cp in data.get("checkpoints", []):
            cp_id = int(cp["id"])
            cp_name = cp.get("display_name", cp.get("name", str(cp_id)))
            CHECKPOINTS[cp_id] = cp_name

        return path

    except Exception as exc:
        print(C.warn(f"WARN checkpoint file not loaded: {exc}"))
        CHECKPOINTS.clear()
        return ""


def format_checkpoint_list() -> str:
    """Return checkpoint list in one compact line."""
    if not CHECKPOINTS:
        return "none"
    return " | ".join(f"{cid}:{name}" for cid, name in sorted(CHECKPOINTS.items()))


def cp_name(cp_id: Optional[int]) -> str:
    if cp_id is None or cp_id < 0:
        return "-"
    return CHECKPOINTS.get(cp_id, str(cp_id))


# ============================================================
#  ROS 2 command sender
# ============================================================
class CommandSender(Node):
    def __init__(self):
        super().__init__("checkpoint_command_sender_v2")

        self._cmd_pub = self.create_publisher(String, "/robot/command", 10)

        self._state = "unknown"
        self._current_cp = -1
        self._status_msg = ""
        self._last_printed_status = ""
        self._warned_no_navigator = False
        self._lock = threading.Lock()

        self.create_subscription(String, "/robot/state", self._on_state, 10)
        self.create_subscription(Int32, "/robot/current_checkpoint", self._on_checkpoint, 10)
        self.create_subscription(String, "/robot/status_message", self._on_status, 10)

    # ------------------------- callbacks -------------------------
    def _on_state(self, msg: String):
        new_state = msg.data.strip() or "unknown"

        with self._lock:
            old_state = self._state
            self._state = new_state
            current_cp = self._current_cp

        # Chỉ in khi state thật sự đổi.
        if new_state != old_state:
            print(self._compact_state_line(new_state, current_cp), flush=True)

    def _on_checkpoint(self, msg: Int32):
        with self._lock:
            self._current_cp = int(msg.data)

    def _on_status(self, msg: String):
        if not PRINT_STATUS_MESSAGE:
            return

        text = msg.data.strip()
        if not text:
            return

        with self._lock:
            if text == self._last_printed_status:
                return
            self._status_msg = text
            self._last_printed_status = text

        # Một dòng duy nhất, không in block dài.
        print(C.dim(f"INFO {text}"), flush=True)

    # ------------------------- display helpers -------------------------
    def _compact_state_line(self, state: str, current_cp: int) -> str:
        color = STATE_COLOR.get(state, lambda s: s)
        hint = STATE_HINTS.get(state, "-")
        cp_text = cp_name(current_cp)
        return f"STATE {color(state):<18} | CP {current_cp if current_cp >= 0 else '-'}:{cp_text} | NEXT {hint}"

    def prompt(self) -> str:
        with self._lock:
            state = self._state
        hint = STATE_HINTS.get(state, "cmd")
        return f"[{state} | {hint}]> "

    # ------------------------- ROS helpers -------------------------
    def _warn_if_no_navigator(self):
        if self._cmd_pub.get_subscription_count() == 0:
            if not WARN_NO_NAVIGATOR_ONCE or not self._warned_no_navigator:
                print(C.warn("WARN navigator not detected on /robot/command"), flush=True)
                self._warned_no_navigator = True

    def wait_for_navigator(self, timeout_s: float = 1.0) -> bool:
        """Wait briefly for navigator_v2.py to appear on /robot/command."""
        start = time.time()
        while rclpy.ok() and time.time() - start < timeout_s:
            if self._cmd_pub.get_subscription_count() > 0:
                return True
            time.sleep(0.05)
        return self._cmd_pub.get_subscription_count() > 0

    def publish_command(self, command: str):
        self._warn_if_no_navigator()
        msg = String()
        msg.data = command
        self._cmd_pub.publish(msg)

    # ------------------------- command helpers -------------------------
    def send_go(self, cp_id: int):
        if CHECKPOINTS and cp_id not in CHECKPOINTS:
            print(C.err(f"ERR invalid checkpoint {cp_id}; valid={sorted(CHECKPOINTS.keys())}"), flush=True)
            return

        name = CHECKPOINTS.get(cp_id, str(cp_id))
        self.publish_command(f"go:{cp_id}")
        print(C.ok(f"SEND go {cp_id} -> {name}"), flush=True)

    def send_stop(self):
        self.publish_command("stop")
        print(C.warn("SEND stop"), flush=True)

    def send_continue(self):
        self.publish_command("continue")
        print(C.ok("SEND continue"), flush=True)

    def send_reset(self):
        self.publish_command("reset")
        print(C.warn("SEND reset"), flush=True)


# ============================================================
#  Terminal UI
# ============================================================
def print_startup(navigator_connected: bool):
    """
    Startup output is intentionally short:
      1 title line
      1 command line
      1 checkpoint line
      1 navigator connection line
    """
    print(C.bold("Checkpoint Commander v2"))
    print("CMD  go <id> | stop | continue | reset | Ctrl+C exit")
    print(f"CP   {format_checkpoint_list()}")
    print(f"NAV  {'connected' if navigator_connected else 'not detected'}")
    print()


def parse_and_send(raw: str, node: CommandSender):
    parts = raw.strip().lower().split()
    if not parts:
        return

    cmd = parts[0]

    if cmd == "go":
        if len(parts) != 2:
            print(C.err("ERR usage: go <id>"), flush=True)
            return
        try:
            node.send_go(int(parts[1]))
        except ValueError:
            print(C.err("ERR checkpoint id must be an integer"), flush=True)
        return

    if cmd == "stop" and len(parts) == 1:
        node.send_stop()
        return

    if cmd == "continue" and len(parts) == 1:
        node.send_continue()
        return

    if cmd == "reset" and len(parts) == 1:
        node.send_reset()
        return

    print(C.err("ERR command must be: go <id> | stop | continue | reset"), flush=True)


def spin_worker(node: Node):
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    except Exception as exc:
        print(C.warn(f"WARN ROS spin: {exc}"), file=sys.stderr)


# ============================================================
#  Main
# ============================================================
def main(args=None):
    rclpy.init(args=args)

    load_checkpoints()
    node = CommandSender()

    spin_thread = threading.Thread(target=spin_worker, args=(node,), daemon=True)
    spin_thread.start()

    navigator_connected = node.wait_for_navigator(timeout_s=1.0)
    print_startup(navigator_connected)

    try:
        while rclpy.ok():
            try:
                raw = input(node.prompt()).strip()
            except EOFError:
                break

            parse_and_send(raw, node)

    except KeyboardInterrupt:
        print("\nexit")

    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
