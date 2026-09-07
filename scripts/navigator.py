#!/usr/bin/env python3
"""
navigator_v2.py
==========================================
Thiết kế lại state machine với 4 lệnh điều khiển rõ ràng hơn v2:

    go <id>   → Điều hướng đến checkpoint
    stop      → Dừng ngay tại chỗ (có thể tiếp tục sau)
    continue  → Tiếp tục hành trình bị dừng
    reset     → Hủy hành trình, chờ 30s rồi tự về Home

Luồng chính:
  IDLE ──go──► COMPUTING_PATH ──► PRE_ROTATING ──► NAVIGATING
                                                        │
                                            succeeded ──┘─► IDLE
                                                              │ (nếu CP ≠ Home)
                                                              ▼
                                                     [15s AT_CHECKPOINT wait]
                                                              │
                                                   go ────────┤─► COMPUTING_PATH
                                                   15s timeout──► RETURNING_HOME
                                            stop     ──────► STOPPED
                                                               │
                                                    continue ──┤─► COMPUTING_PATH
                                                    reset    ──┘─► WAITING_RESET
                                                                        │
                                                            go ─────────┤─► COMPUTING_PATH
                                                            30s timeout ──► RETURNING_HOME

Thay đổi so với v2:
  - EMERGENCY_STOP → STOPPED (có thể resume)
  - AT_CHECKPOINT bị xóa → sau khi đến CP thì về IDLE ngay
  - Thêm WAITING_RESET (30s chờ sau reset)
  - Thêm at_checkpoint timer 15s: sau khi đến CP (≠ Home) thành công,
    chờ 15s. Nếu nhận go:<id> → hủy timer; nếu hết 15s → tự về Home.
  - Dùng 1 topic /robot/command (String) thay vì nhiều topic
  - _stop_requested flag: chặn _send_nav_goal khi stop xảy ra trong COMPUTING_PATH
  - _intentional_cancel flag: phân biệt cancel chủ ý vs Nav2-triggered

Topics:
  Sub: /robot/command  (std_msgs/String) — "go:1" | "stop" | "continue" | "reset"
  Pub: /robot/state    (std_msgs/String)
  Pub: /robot/current_checkpoint (std_msgs/Int32)
  Pub: /robot/status_message     (std_msgs/String)
  Pub: /cmd_vel        (geometry_msgs/Twist) — chỉ trong PRE_ROTATING
"""

import math
import os
import time
from enum import Enum

import rclpy
import rclpy.time
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node

import tf2_ros
import yaml

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import Path
from std_msgs.msg import Int32, String


# ============================================================
#  TUNING PARAMETERS
# ============================================================

# Pre-rotate
PRE_ROTATE_THRESHOLD = 0.40     # rad (~23°) — ngưỡng kích hoạt pre-rotate
PRE_ROTATE_STOP_THR  = 0.05     # rad (~3°)  — ngưỡng coi là đã căn xong
PRE_ROTATE_LOOKAHEAD = 0.50     # m   — lookahead trên path để tính heading ban đầu
PRE_ROTATE_KP        = 0.8      # P-gain bộ điều khiển xoay tại chỗ
PRE_ROTATE_MAX_W     = 0.8      # rad/s — tốc độ góc tối đa
PRE_ROTATE_MIN_W     = 0.4      # rad/s — tốc độ góc tối thiểu (thắng ma sát tĩnh)
PRE_ROTATE_TIMEOUT   = 12.0     # s  — timeout pre-rotate

# Home retry khi RETURNING_HOME bị abort
HOME_RETRY_MAX       = 3        # lần retry tối đa
HOME_RETRY_DELAY_S   = 5.0      # s chờ giữa các lần retry

# Timeout chờ Nav2 action servers
NAV2_SERVER_TIMEOUT_S = 30.0    # s

# Thời gian chờ ở WAITING_RESET trước khi tự về Home
RESET_WAIT_TIMEOUT_S  = 30      # s (int — đếm bằng 1s tick)

# Thời gian chờ tại checkpoint trước khi tự về Home
# Robot ở trạng thái IDLE trong suốt khoảng chờ này → lệnh go:<id> vẫn hợp lệ.
AT_CHECKPOINT_WAIT_S  = 15      # s (int — đếm bằng 1s tick)

# ============================================================


class State(Enum):
    IDLE           = "IDLE"
    COMPUTING_PATH = "COMPUTING_PATH"
    PRE_ROTATING   = "PRE_ROTATING"
    NAVIGATING     = "NAVIGATING"
    STOPPED        = "STOPPED"          # ← Thay thế EMERGENCY_STOP + AT_CHECKPOINT
    WAITING_RESET  = "WAITING_RESET"    # ← Mới: chờ 30s sau lệnh reset
    RETURNING_HOME = "RETURNING_HOME"


# Các state mà robot "đang bận" → chặn lệnh go
_BUSY_STATES = {
    State.COMPUTING_PATH,
    State.PRE_ROTATING,
    State.NAVIGATING,
    State.RETURNING_HOME,
}

# Các state mà lệnh go hợp lệ
_GO_VALID_STATES = {State.IDLE, State.WAITING_RESET}

# Các state mà lệnh stop hợp lệ
_STOP_VALID_STATES = {State.COMPUTING_PATH, State.PRE_ROTATING, State.NAVIGATING}


class CheckpointNavigator(Node):
    """
    Navigator v2: 4-command state machine với debug log đầy đủ.
    Thêm tính năng: sau khi đến checkpoint (≠ Home) thành công,
    chờ AT_CHECKPOINT_WAIT_S giây rồi tự về Home nếu không nhận lệnh go.
    """

    def __init__(self):
        super().__init__("checkpoint_navigator")

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter("checkpoint_file",    "")
        self.declare_parameter("home_checkpoint_id", 0)

        self.home_id = self.get_parameter("home_checkpoint_id").value

        self.checkpoints = self._load_checkpoints()
        if not self.checkpoints:
            self.get_logger().error("No checkpoints loaded. Navigator exiting.")
            return

        # ── State variables ──────────────────────────────────────────────────
        self.state        = State.IDLE
        self.current_cp   = -1          # ID của CP đã đến gần nhất (thành công)
        self.target_cp    = None        # ID CP đang hướng đến
        self.goal_handle  = None        # NavigateToPose goal handle

        # Pre-rotate
        self._target_yaw       = 0.0
        self._pre_rotate_start = None
        self._pending_cp_id    = None

        # Stop/continue/reset flags
        self._stop_requested    = False  # Chặn _send_nav_goal khi stop trong COMPUTING_PATH
        self._intentional_cancel = False # Phân biệt cancel chủ ý vs Nav2-triggered
        self._saved_target_cp   = None   # CP được lưu khi stop, dùng cho continue

        # Returning home
        self._is_returning_home = False
        self._home_retry_count  = 0

        # WAITING_RESET timer state
        self._reset_elapsed = 0
        self._reset_timer   = None   # timer handle

        # ── AT_CHECKPOINT wait-then-return-home timer ────────────────────────
        # Được kích hoạt sau khi đến CP ≠ Home thành công.
        # Robot ở State.IDLE trong suốt thời gian chờ.
        self._at_cp_elapsed = 0
        self._at_cp_timer   = None   # timer handle

        # ── TF ──────────────────────────────────────────────────────────────
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ── Action clients ───────────────────────────────────────────────────
        self._nav     = ActionClient(self, NavigateToPose,    "navigate_to_pose")
        self._planner = ActionClient(self, ComputePathToPose, "compute_path_to_pose")

        self.get_logger().info(
            f"Waiting for Nav2 action servers (timeout={NAV2_SERVER_TIMEOUT_S:.0f}s)...")
        nav_ok     = self._nav.wait_for_server(timeout_sec=NAV2_SERVER_TIMEOUT_S)
        planner_ok = self._planner.wait_for_server(timeout_sec=NAV2_SERVER_TIMEOUT_S)

        if not nav_ok:
            raise RuntimeError(
                f"navigate_to_pose not available after {NAV2_SERVER_TIMEOUT_S:.0f}s. "
                "Is Nav2 running?")
        if not planner_ok:
            raise RuntimeError(
                f"compute_path_to_pose not available after {NAV2_SERVER_TIMEOUT_S:.0f}s. "
                "Is planner_server running?")

        self.get_logger().info("Nav2 action servers ready ✓")

        # ── Publishers ───────────────────────────────────────────────────────
        self._state_pub  = self.create_publisher(String, "/robot/state",              10)
        self._cp_pub     = self.create_publisher(Int32,  "/robot/current_checkpoint", 10)
        self._status_pub = self.create_publisher(String, "/robot/status_message",     10)
        self._cmdvel_pub = self.create_publisher(Twist,  "/cmd_vel",                  10)

        # ── Subscribers ──────────────────────────────────────────────────────
        self.create_subscription(String, "/robot/command", self._on_command, 10)

        # ── Timers ───────────────────────────────────────────────────────────
        self.create_timer(0.1, self._state_machine_tick)  # 10 Hz — pre-rotate loop
        self.create_timer(1.0, self._publish_state)       # 1 Hz  — state broadcast

        self.get_logger().info(
            f"CheckpointNavigator v2 ready │ "
            f"{len(self.checkpoints)} checkpoints │ "
            f"home_id={self.home_id} │ "
            f"pre_rotate_thr={math.degrees(PRE_ROTATE_THRESHOLD):.0f}° │ "
            f"reset_wait={RESET_WAIT_TIMEOUT_S}s │ "
            f"at_checkpoint_wait={AT_CHECKPOINT_WAIT_S}s"
        )

    # ════════════════════════════════════════════════════════
    #  CHECKPOINT LOADING
    # ════════════════════════════════════════════════════════

    def _load_checkpoints(self) -> dict:
        path = self.get_parameter("checkpoint_file").value
        if not path or not os.path.exists(path):
            try:
                pkg  = get_package_share_directory("mobile_robot")
                path = os.path.join(pkg, "config", "checkpoints_v2.yaml")
            except Exception:
                self.get_logger().error("Package 'mobile_robot' not found.")
                return {}

        if not os.path.exists(path):
            self.get_logger().error(f"Checkpoint file not found: {path}")
            return {}

        with open(path, "r") as f:
            data = yaml.safe_load(f)

        result = {}
        for cp in data.get("checkpoints", []):
            pose = PoseStamped()
            pose.header.frame_id    = data.get("frame_id", "map")
            pose.pose.position.x    = float(cp["position"]["x"])
            pose.pose.position.y    = float(cp["position"]["y"])
            pose.pose.position.z    = float(cp["position"].get("z", 0.0))
            pose.pose.orientation.x = float(cp["orientation"].get("x", 0.0))
            pose.pose.orientation.y = float(cp["orientation"].get("y", 0.0))
            pose.pose.orientation.z = float(cp["orientation"].get("z", 0.0))
            pose.pose.orientation.w = float(cp["orientation"].get("w", 1.0))
            result[cp["id"]] = {
                "id":   cp["id"],
                "name": cp.get("display_name", cp["name"]),
                "pose": pose,
            }
        self.get_logger().info(
            f"Loaded {len(result)} checkpoints from {path}")
        return result

    # ════════════════════════════════════════════════════════
    #  COMMAND DISPATCHER
    # ════════════════════════════════════════════════════════

    def _on_command(self, msg: String):
        """
        Nhận lệnh từ /robot/command.
        Định dạng: "go:1" | "stop" | "continue" | "reset"
        """
        raw = msg.data.strip().lower()
        self.get_logger().info(
            f"[CMD] Received: '{raw}' │ Current state: {self.state.name}")

        if raw.startswith("go:"):
            self._cmd_go(raw)
        elif raw == "stop":
            self._cmd_stop()
        elif raw == "continue":
            self._cmd_continue()
        elif raw == "reset":
            self._cmd_reset()
        else:
            self.get_logger().warn(
                f"[CMD] Unknown command: '{raw}'. "
                "Valid: 'go:<id>', 'stop', 'continue', 'reset'")

    # ════════════════════════════════════════════════════════
    #  COMMAND: go
    # ════════════════════════════════════════════════════════

    def _cmd_go(self, raw: str):
        """
        go:<id> — Điều hướng đến checkpoint id.
        Hợp lệ trong: IDLE (kể cả đang chờ at_checkpoint timer), WAITING_RESET
        """
        # Parse ID
        try:
            cp_id = int(raw.split(":")[1])
        except (IndexError, ValueError):
            self.get_logger().error(
                f"[GO] Invalid format '{raw}'. Expected 'go:<int>'")
            return

        # Validate checkpoint
        if cp_id not in self.checkpoints:
            self.get_logger().error(
                f"[GO] Checkpoint {cp_id} not found. "
                f"Valid: {sorted(self.checkpoints.keys())}")
            return

        # Check state
        if self.state in _BUSY_STATES:
            self.get_logger().warn(
                f"[GO] Rejected: robot busy in {self.state.name} → "
                f"navigating to [{self.target_cp}]. Use 'stop' first.")
            self._pub_status(
                f"Busy ({self.state.name}). Use 'stop' first.")
            return

        if self.state == State.STOPPED:
            self.get_logger().warn(
                "[GO] Rejected: robot is STOPPED. Use 'continue' or 'reset'.")
            self._pub_status("Robot STOPPED. Use 'continue' or 'reset'.")
            return

        if self.state not in _GO_VALID_STATES:
            self.get_logger().warn(
                f"[GO] Rejected: not valid in state {self.state.name}.")
            return

        # ── Hủy at_checkpoint timer nếu đang đếm ngược ──────────────────────
        # Điều này xảy ra khi người dùng gửi go:<id> trong 15s chờ sau khi
        # robot đến CP thành công (robot đang ở IDLE + _at_cp_timer đang chạy).
        self._cancel_at_cp_timer()

        # WAITING_RESET → hủy timer 30s trước khi thực thi
        if self.state == State.WAITING_RESET:
            self._cancel_reset_timer()
            self.get_logger().info(
                f"[GO] Received during WAITING_RESET → canceling reset timer.")

        # Nếu đã ở đúng CP và đang IDLE (và không có at_cp_timer nào chạy)
        if self.state == State.IDLE and self.current_cp == cp_id:
            self.get_logger().info(
                f"[GO] Already at checkpoint [{cp_id}] "
                f"'{self.checkpoints[cp_id]['name']}'.")
            return

        self.get_logger().info(
            f"[GO] ✓ Navigating to [{cp_id}] "
            f"'{self.checkpoints[cp_id]['name']}'")

        # Reset returning-home flags khi nhận lệnh go thủ công
        self._is_returning_home = False
        self._home_retry_count  = 0
        self._stop_requested    = False

        self._request_navigation(cp_id)

    # ════════════════════════════════════════════════════════
    #  COMMAND: stop
    # ════════════════════════════════════════════════════════

    def _cmd_stop(self):
        """
        stop — Dừng ngay, lưu lại target để có thể continue.
        Hợp lệ trong: COMPUTING_PATH, PRE_ROTATING, NAVIGATING
        """
        if self.state not in _STOP_VALID_STATES:
            self.get_logger().warn(
                f"[STOP] Rejected: not valid in state {self.state.name}. "
                f"Valid states: {[s.name for s in _STOP_VALID_STATES]}")
            return

        # Lưu checkpoint đang nhắm tới
        self._saved_target_cp = self.target_cp
        saved_name = self.checkpoints.get(self._saved_target_cp, {}).get("name", "?")

        self.get_logger().info(
            f"[STOP] ✓ Stopping. Saving target=[{self._saved_target_cp}] "
            f"'{saved_name}'")

        # Đặt flag TRƯỚC KHI cancel — _on_result sẽ kiểm tra flag này
        self._intentional_cancel = True

        # Chặn _send_nav_goal nếu đang trong COMPUTING_PATH/PRE_ROTATING
        self._stop_requested = True

        # Dừng chuyển động ngay
        self._stop_robot()

        # Cancel Nav2 goal nếu đang NAVIGATING
        if self.state == State.NAVIGATING and self.goal_handle is not None:
            cancel_future = self.goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self._on_cancel_done)
            self.goal_handle = None
            self.get_logger().info(
                "[STOP] Nav2 goal cancel sent.")

        self.state = State.STOPPED
        self._pub_status(
            f"STOPPED. Target [{self._saved_target_cp}] '{saved_name}' saved. "
            "Use 'continue' or 'reset'.")

    def _on_cancel_done(self, future):
        """Callback sau khi Nav2 xác nhận đã cancel goal."""
        try:
            result = future.result()
            self.get_logger().info(
                f"[STOP] Nav2 cancel confirmed. Goals canceled: "
                f"{len(result.goals_canceling)}")
        except Exception as e:
            self.get_logger().warn(f"[STOP] Cancel callback error: {e}")

    # ════════════════════════════════════════════════════════
    #  COMMAND: continue
    # ════════════════════════════════════════════════════════

    def _cmd_continue(self):
        """
        continue — Tiếp tục hành trình bị dừng.
        Hợp lệ trong: STOPPED
        Re-compute path (vì robot có thể đã bị di chuyển tay) → PRE_ROTATING.
        """
        if self.state != State.STOPPED:
            self.get_logger().warn(
                f"[CONTINUE] Rejected: only valid in STOPPED, "
                f"current={self.state.name}")
            return

        if self._saved_target_cp is None:
            self.get_logger().error(
                "[CONTINUE] No saved target checkpoint. Use 'go:<id>' instead.")
            return

        cp_id = self._saved_target_cp
        saved_name = self.checkpoints.get(cp_id, {}).get("name", "?")

        self.get_logger().info(
            f"[CONTINUE] ✓ Re-planning to [{cp_id}] '{saved_name}' "
            "(full re-compute including pre-rotate)")

        # Reset flags
        self._stop_requested     = False
        self._intentional_cancel = False
        self._saved_target_cp    = None

        self._pub_status(
            f"Continuing to [{cp_id}] '{saved_name}'...")
        self._request_navigation(cp_id)

    # ════════════════════════════════════════════════════════
    #  COMMAND: reset
    # ════════════════════════════════════════════════════════

    def _cmd_reset(self):
        """
        reset — Hủy hành trình, vào WAITING_RESET.
        Hợp lệ trong: STOPPED
        Sau 30s tự về Home; nếu nhận 'go:<id>' trong 30s → điều hướng đến id đó.
        """
        if self.state != State.STOPPED:
            self.get_logger().warn(
                f"[RESET] Rejected: only valid in STOPPED, "
                f"current={self.state.name}")
            return

        self.get_logger().info(
            f"[RESET] ✓ Entering WAITING_RESET. "
            f"Will go home [{self.home_id}] in {RESET_WAIT_TIMEOUT_S}s "
            "unless 'go:<id>' is received.")

        # Xóa saved target
        self._saved_target_cp    = None
        self._stop_requested     = False
        self._intentional_cancel = False

        self.state = State.WAITING_RESET
        self._pub_status(
            f"WAITING_RESET: send 'go:<id>' within {RESET_WAIT_TIMEOUT_S}s, "
            f"or robot will return home [{self.home_id}].")

        # Khởi động one-shot timer 30s
        self._start_reset_timer()

    def _start_reset_timer(self):
        """Khởi động bộ đếm ngược RESET_WAIT_TIMEOUT_S giây (pattern 1s tick)."""
        self._reset_elapsed = 0
        self._reset_timer   = self.create_timer(1.0, self._reset_tick)
        self.get_logger().info(
            f"[RESET_TIMER] Started. Countdown: {RESET_WAIT_TIMEOUT_S}s")

    def _reset_tick(self):
        """Tick 1s — đếm ngược. Khi hết giờ → về Home."""
        # Nếu state đã thay đổi (nhận lệnh go), timer tự hủy
        if self.state != State.WAITING_RESET:
            self._reset_timer.cancel()
            self._reset_timer = None
            self.get_logger().info("[RESET_TIMER] Canceled (state changed).")
            return

        self._reset_elapsed += 1
        remaining = RESET_WAIT_TIMEOUT_S - self._reset_elapsed

        # Log mỗi 15s để không spam terminal
        if remaining % 15 == 0 or remaining <= 5:
            self.get_logger().info(
                f"[RESET_TIMER] {remaining}s remaining before going home [{self.home_id}].")

        if self._reset_elapsed >= RESET_WAIT_TIMEOUT_S:
            self._reset_timer.cancel()
            self._reset_timer = None
            self.get_logger().info(
                f"[RESET_TIMER] Timeout! Returning home [{self.home_id}].")
            self._pub_status(
                f"Reset timeout. Returning home [{self.home_id}]...")
            self._is_returning_home = True
            self._home_retry_count  = 0
            self._request_navigation(self.home_id)

    def _cancel_reset_timer(self):
        """Hủy timer WAITING_RESET nếu đang chạy."""
        if self._reset_timer is not None:
            self._reset_timer.cancel()
            self._reset_timer = None
            self.get_logger().info(
                "[RESET_TIMER] Canceled (new go command received).")

    # ════════════════════════════════════════════════════════
    #  AT_CHECKPOINT wait-then-return-home timer
    # ════════════════════════════════════════════════════════

    def _start_at_cp_timer(self, cp_id: int):
        """
        Khởi động bộ đếm ngược AT_CHECKPOINT_WAIT_S giây.
        Được gọi sau khi đến CP ≠ Home thành công.
        Robot ở State.IDLE trong suốt khoảng chờ.
        """
        self._at_cp_elapsed = 0
        self._at_cp_timer   = self.create_timer(1.0, self._at_cp_tick)
        cp_name = self.checkpoints.get(cp_id, {}).get("name", str(cp_id))
        self.get_logger().info(
            f"[AT_CP_TIMER] Started at [{cp_id}] '{cp_name}'. "
            f"Will return home [{self.home_id}] in {AT_CHECKPOINT_WAIT_S}s "
            "unless 'go:<id>' is received.")
        self._pub_status(
            f"At [{cp_id}] '{cp_name}'. "
            f"Send 'go:<id>' within {AT_CHECKPOINT_WAIT_S}s or robot returns home.")

    def _at_cp_tick(self):
        """
        Tick 1s — đếm ngược AT_CHECKPOINT_WAIT_S.
        Khi hết giờ → set _is_returning_home và navigate về Home.
        Nếu state không còn là IDLE (robot bị điều khiển đi nơi khác) → hủy.
        """
        # Guard: nếu robot rời khỏi IDLE (ví dụ đã nhận go và đang navigate),
        # timer tự hủy để không gây xung đột.
        if self.state != State.IDLE:
            self._at_cp_timer.cancel()
            self._at_cp_timer = None
            self.get_logger().info(
                "[AT_CP_TIMER] Canceled (robot left IDLE state).")
            return

        self._at_cp_elapsed += 1
        remaining = AT_CHECKPOINT_WAIT_S - self._at_cp_elapsed

        # Log mỗi 15s và 3s cuối để không spam terminal
        if remaining % 15 == 0 or remaining <= 3:
            self.get_logger().info(
                f"[AT_CP_TIMER] {remaining}s remaining before returning home [{self.home_id}].")

        if self._at_cp_elapsed >= AT_CHECKPOINT_WAIT_S:
            self._at_cp_timer.cancel()
            self._at_cp_timer = None
            self.get_logger().info(
                f"[AT_CP_TIMER] Timeout! Returning home [{self.home_id}].")
            self._pub_status(
                f"Checkpoint wait timeout. Returning home [{self.home_id}]...")
            self._is_returning_home = True
            self._home_retry_count  = 0
            self._request_navigation(self.home_id)

    def _cancel_at_cp_timer(self):
        """
        Hủy at_checkpoint timer nếu đang chạy.
        Được gọi từ _cmd_go() khi người dùng gửi go:<id> trong 15s chờ.
        """
        if self._at_cp_timer is not None:
            self._at_cp_timer.cancel()
            self._at_cp_timer = None
            self.get_logger().info(
                "[AT_CP_TIMER] Canceled (new go command received).")

    # ════════════════════════════════════════════════════════
    #  STATE MACHINE TICK (10 Hz)
    # ════════════════════════════════════════════════════════

    def _state_machine_tick(self):
        """Main 10 Hz loop — chỉ xử lý PRE_ROTATING tick."""
        if self.state == State.PRE_ROTATING:
            self._pre_rotate_tick()

    # ════════════════════════════════════════════════════════
    #  NAVIGATION PIPELINE
    # ════════════════════════════════════════════════════════

    def _request_navigation(self, cp_id: int):
        """
        Entry point cho mọi navigation request.
        Bước 1: Gọi ComputePathToPose để lấy path → trích heading ban đầu.
        """
        self._pending_cp_id = cp_id
        self.target_cp      = cp_id
        self.state          = State.COMPUTING_PATH

        cp_name = self.checkpoints[cp_id]["name"]
        self.get_logger().info(
            f"[COMPUTING_PATH] → [{cp_id}] '{cp_name}' │ "
            f"is_returning_home={self._is_returning_home}")
        self._pub_status(f"Computing path to [{cp_id}] '{cp_name}'...")

        goal_pose              = self.checkpoints[cp_id]["pose"]
        goal_pose.header.stamp = self.get_clock().now().to_msg()

        compute_goal            = ComputePathToPose.Goal()
        compute_goal.pose       = goal_pose
        compute_goal.planner_id = ""  # dùng default planner (NavFn/A*)

        future = self._planner.send_goal_async(compute_goal)
        future.add_done_callback(self._on_path_goal_response)

    # ─── Step 1: Planner accepted/rejected ────────────────────────────────

    def _on_path_goal_response(self, future):
        """Planner phản hồi chấp nhận/từ chối goal."""
        # Guard: stop đã được gọi trong lúc chờ planner
        if self._stop_requested:
            self.get_logger().info(
                "[COMPUTING_PATH] stop_requested → discarding path goal response. "
                "State already STOPPED.")
            return

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(
                "[COMPUTING_PATH] Planner REJECTED goal. "
                "Falling back to direct NavigateToPose (no pre-rotate).")
            self._send_nav_goal(self._pending_cp_id)
            return

        goal_handle.get_result_async().add_done_callback(self._on_path_result)

    # ─── Step 2: Planner trả về path ──────────────────────────────────────

    def _on_path_result(self, future):
        """Phân tích path, quyết định có cần pre-rotate không."""
        # Guard: stop trong lúc planner tính toán
        if self._stop_requested:
            self.get_logger().info(
                "[COMPUTING_PATH] stop_requested → discarding path result. "
                "Will NOT send NavigateToPose.")
            return

        result = future.result()

        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(
                f"[COMPUTING_PATH] Planner failed (status={result.status}). "
                "Falling back to direct nav.")
            self._send_nav_goal(self._pending_cp_id)
            return

        path: Path = result.result.path

        if len(path.poses) < 2:
            self.get_logger().warn(
                "[COMPUTING_PATH] Path < 2 poses (goal very close?). "
                "Skipping pre-rotate.")
            self._send_nav_goal(self._pending_cp_id)
            return

        # Lấy pose robot hiện tại
        robot_pose = self._get_robot_pose()
        if robot_pose is None:
            self.get_logger().warn(
                "[COMPUTING_PATH] TF unavailable → Skipping pre-rotate.")
            self._send_nav_goal(self._pending_cp_id)
            return

        rx, ry, ryaw = robot_pose
        initial_heading = self._extract_initial_heading(path, rx, ry)
        heading_error   = self._normalize_angle(initial_heading - ryaw)

        self.get_logger().info(
            f"[COMPUTING_PATH] Path heading: {math.degrees(initial_heading):+.1f}° │ "
            f"Robot yaw: {math.degrees(ryaw):+.1f}° │ "
            f"Error: {math.degrees(heading_error):+.1f}° "
            f"(threshold: ±{math.degrees(PRE_ROTATE_THRESHOLD):.1f}°)")

        if abs(heading_error) < PRE_ROTATE_THRESHOLD:
            self.get_logger().info(
                "[COMPUTING_PATH] Heading error within threshold → Skipping pre-rotate.")
            self._send_nav_goal(self._pending_cp_id)
            return

        # Kích hoạt pre-rotate
        self._target_yaw       = initial_heading
        self._pre_rotate_start = time.monotonic()
        self.state             = State.PRE_ROTATING

        self.get_logger().info(
            f"[PRE_ROTATING] Rotating {math.degrees(heading_error):+.1f}° "
            f"to align with path tangent...")
        self._pub_status(
            f"Pre-rotating {math.degrees(heading_error):+.0f}° "
            f"→ [{self._pending_cp_id}] '{self.checkpoints[self._pending_cp_id]['name']}'")

    # ─── Pre-rotate control loop ──────────────────────────────────────────

    def _pre_rotate_tick(self):
        """Chạy 10 Hz khi state == PRE_ROTATING."""
        # Guard: stop trong lúc pre-rotating
        if self._stop_requested:
            self._stop_robot()
            self.get_logger().info(
                "[PRE_ROTATING] stop_requested → aborting pre-rotate. "
                "State already STOPPED.")
            return

        elapsed = time.monotonic() - self._pre_rotate_start
        if elapsed > PRE_ROTATE_TIMEOUT:
            self.get_logger().warn(
                f"[PRE_ROTATING] Timeout ({PRE_ROTATE_TIMEOUT:.0f}s). "
                "Proceeding to navigate anyway.")
            self._stop_robot()
            self._send_nav_goal(self._pending_cp_id)
            return

        robot_pose = self._get_robot_pose()
        if robot_pose is None:
            return  # TF chưa sẵn sàng, giữ cmd_vel cũ và chờ

        _, _, ryaw    = robot_pose
        heading_error = self._normalize_angle(self._target_yaw - ryaw)

        # Kiểm tra hội tụ
        if abs(heading_error) < PRE_ROTATE_STOP_THR:
            self.get_logger().info(
                f"[PRE_ROTATING] ✓ Converged │ "
                f"residual={math.degrees(heading_error):+.2f}° │ "
                f"elapsed={elapsed:.1f}s")
            self._stop_robot()
            self._send_nav_goal(self._pending_cp_id)
            return

        # P controller
        raw_w = PRE_ROTATE_KP * heading_error
        sign  = 1.0 if raw_w >= 0.0 else -1.0
        w     = sign * max(PRE_ROTATE_MIN_W, min(PRE_ROTATE_MAX_W, abs(raw_w)))

        twist = Twist()
        twist.angular.z = w
        self._cmdvel_pub.publish(twist)

        # Debug log mỗi 1s
        if int(elapsed / 0.1) % 10 == 0:
            self.get_logger().debug(
                f"[PRE_ROTATING] err={math.degrees(heading_error):+.1f}° │ "
                f"w={w:+.2f} rad/s │ elapsed={elapsed:.1f}s")

    # ─── Send NavigateToPose goal ─────────────────────────────────────────

    def _send_nav_goal(self, cp_id: int):
        """
        Gửi NavigateToPose đến Nav2.
        Đây là điểm cuối của pipeline — guard stop_requested trước khi gửi.
        """
        # Guard: stop đã được yêu cầu → không gửi
        if self._stop_requested:
            self.get_logger().warn(
                f"[NAVIGATING] stop_requested → blocked NavigateToPose to [{cp_id}].")
            return

        self.target_cp = cp_id
        self.state     = State.NAVIGATING

        pose              = self.checkpoints[cp_id]["pose"]
        pose.header.stamp = self.get_clock().now().to_msg()

        goal      = NavigateToPose.Goal()
        goal.pose = pose

        cp_name = self.checkpoints[cp_id]["name"]
        self.get_logger().info(
            f"[NAVIGATING] ► [{cp_id}] '{cp_name}' │ "
            f"x={pose.pose.position.x:.2f} y={pose.pose.position.y:.2f} │ "
            f"is_returning_home={self._is_returning_home}")
        self._pub_status(f"Navigating → [{cp_id}] '{cp_name}'")

        self._nav.send_goal_async(goal).add_done_callback(self._on_goal_accepted)

    def _on_goal_accepted(self, future):
        """Nav2 đã chấp nhận/từ chối goal."""
        handle = future.result()

        # Guard: stop xảy ra giữa lúc gửi goal và callback này
        if self._stop_requested:
            self.get_logger().warn(
                "[NAVIGATING] stop_requested after goal sent → canceling accepted goal.")
            if handle.accepted:
                handle.cancel_goal_async()
            return

        self.goal_handle = handle
        if not self.goal_handle.accepted:
            self.get_logger().error(
                "[NAVIGATING] Nav2 REJECTED NavigateToPose goal!")
            self.state = State.IDLE
            self._pub_status("Goal rejected by Nav2. Robot IDLE.")
            return

        self.get_logger().info(
            f"[NAVIGATING] Nav2 accepted goal to [{self.target_cp}].")
        self.goal_handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        """Nhận kết quả sau khi Nav2 hoàn thành (succeed/canceled/aborted)."""
        status = future.result().status
        cp_id  = self.target_cp
        cp_name = self.checkpoints.get(cp_id, {}).get("name", "?")

        self.get_logger().info(
            f"[RESULT] Goal to [{cp_id}] '{cp_name}' finished │ "
            f"status={status} │ "
            f"is_returning_home={self._is_returning_home} │ "
            f"intentional_cancel={self._intentional_cancel}")

        # ── SUCCEEDED ────────────────────────────────────────────────────
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.current_cp        = cp_id
            self._home_retry_count = 0

            if self._is_returning_home:
                # ── Đến Home sau khi tự về (RETURNING_HOME) ─────────────
                # Không tạo at_checkpoint timer — robot nghỉ tại Home.
                self._is_returning_home = False
                self.state = State.IDLE
                self.get_logger().info(
                    f"[RESULT] ✓ Arrived at HOME [{cp_id}] '{cp_name}'. "
                    "State: IDLE.")
                self._pub_status(
                    f"At Home [{cp_id}] '{cp_name}'. IDLE.")

            elif cp_id == self.home_id:
                # ── Đến Home theo lệnh go thủ công ───────────────────────
                # Người dùng chủ động điều hướng về Home → không tạo timer,
                # robot nghỉ tại Home chờ lệnh tiếp theo.
                self.state = State.IDLE
                self.get_logger().info(
                    f"[RESULT] ✓ Arrived at HOME [{cp_id}] '{cp_name}' "
                    "(manual go). State: IDLE.")
                self._pub_status(
                    f"Arrived at Home [{cp_id}] '{cp_name}'. "
                    "Send 'go:<id>' for next destination.")

            else:
                # ── Đến checkpoint thông thường (≠ Home) ─────────────────
                # Chuyển IDLE ngay, đồng thời khởi động at_checkpoint timer.
                # Trong AT_CHECKPOINT_WAIT_S giây:
                #   • Nếu nhận go:<id> → _cancel_at_cp_timer() rồi navigate.
                #   • Nếu hết giờ    → _at_cp_tick() tự navigate về Home.
                self.state = State.IDLE
                self.get_logger().info(
                    f"[RESULT] ✓ Arrived at [{cp_id}] '{cp_name}'. "
                    f"State: IDLE. Starting {AT_CHECKPOINT_WAIT_S}s wait timer.")
                self._start_at_cp_timer(cp_id)

        # ── CANCELED ─────────────────────────────────────────────────────
        elif status == GoalStatus.STATUS_CANCELED:
            if self._intentional_cancel:
                # Cancel chủ ý từ lệnh stop → không đổi state (đã set STOPPED)
                self._intentional_cancel = False
                self.get_logger().info(
                    "[RESULT] Intentional cancel confirmed. State remains STOPPED.")
            else:
                # Cancel ngoài ý muốn (Nav2 tự cancel)
                self.get_logger().warn(
                    "[RESULT] Unexpected cancel from Nav2. State → IDLE.")
                self._is_returning_home = False
                self._home_retry_count  = 0
                self.current_cp         = -1
                self.state              = State.IDLE
                self._pub_status("Navigation canceled unexpectedly. IDLE.")

        # ── ABORTED ──────────────────────────────────────────────────────
        elif status == GoalStatus.STATUS_ABORTED:
            if self._is_returning_home and self._home_retry_count < HOME_RETRY_MAX:
                self._home_retry_count += 1
                self.get_logger().warn(
                    f"[RESULT] RETURNING_HOME aborted │ "
                    f"Retry {self._home_retry_count}/{HOME_RETRY_MAX} "
                    f"in {HOME_RETRY_DELAY_S:.0f}s...")
                self._pub_status(
                    f"Home path blocked. Retry {self._home_retry_count}/{HOME_RETRY_MAX} "
                    f"in {HOME_RETRY_DELAY_S:.0f}s.")
                self.create_timer(HOME_RETRY_DELAY_S, self._retry_home_once)
            else:
                if self._is_returning_home:
                    self.get_logger().error(
                        f"[RESULT] CRITICAL: Cannot reach home after "
                        f"{HOME_RETRY_MAX} retries. Manual intervention needed!")
                    self._pub_status(
                        f"CRITICAL: Cannot reach home [{self.home_id}]. "
                        "Please manually move robot.")
                else:
                    self.get_logger().error(
                        f"[RESULT] Navigation to [{cp_id}] '{cp_name}' ABORTED. "
                        "Path blocked or planner failed.")
                    self._pub_status(
                        f"Aborted. Could not reach [{cp_id}] '{cp_name}'. IDLE.")

                self._is_returning_home = False
                self._home_retry_count  = 0
                self.current_cp         = -1
                self.state              = State.IDLE

    def _retry_home_once(self):
        """Timer callback cho home retry."""
        if not self._is_returning_home or self.state == State.STOPPED:
            return
        self.get_logger().info(
            f"[HOME_RETRY] Attempt {self._home_retry_count}/{HOME_RETRY_MAX} → "
            f"home [{self.home_id}]")
        self._request_navigation(self.home_id)

    # ════════════════════════════════════════════════════════
    #  UTILITY METHODS
    # ════════════════════════════════════════════════════════

    def _extract_initial_heading(self, path: Path, rx: float, ry: float) -> float:
        """Trả về heading (rad, map frame) từ robot đến điểm lookahead trên path."""
        poses = path.poses

        if len(poses) == 1:
            px = poses[0].pose.position.x
            py = poses[0].pose.position.y
            return math.atan2(py - ry, px - rx)

        for pose_stamped in poses:
            px   = pose_stamped.pose.position.x
            py   = pose_stamped.pose.position.y
            dist = math.hypot(px - rx, py - ry)
            if dist >= PRE_ROTATE_LOOKAHEAD:
                return math.atan2(py - ry, px - rx)

        # Tất cả điểm đều gần hơn lookahead → dùng điểm cuối
        last = poses[-1].pose.position
        return math.atan2(last.y - ry, last.x - rx)

    def _get_robot_pose(self):
        """Trả về (x, y, yaw_rad) của base_footprint trong frame map. None nếu lỗi."""
        try:
            tf = self._tf_buffer.lookup_transform(
                "map", "base_footprint",
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1)
            )
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(
                f"TF error: {e}", throttle_duration_sec=2.0)
            return None

        x = tf.transform.translation.x
        y = tf.transform.translation.y
        q = tf.transform.rotation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return x, y, math.atan2(siny, cosy)

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Wrap to (−π, π]."""
        while angle >  math.pi:
            angle -= 2.0 * math.pi
        while angle <= -math.pi:
            angle += 2.0 * math.pi
        return angle

    def _stop_robot(self):
        """Gửi Twist(0) để dừng robot ngay."""
        self._cmdvel_pub.publish(Twist())

    # ════════════════════════════════════════════════════════
    #  STATE PUBLISHERS
    # ════════════════════════════════════════════════════════

    def _publish_state(self):
        """1 Hz — broadcast state và current checkpoint."""
        m = String(); m.data = self.state.value;  self._state_pub.publish(m)
        m = Int32();  m.data = self.current_cp;   self._cp_pub.publish(m)

    def _pub_status(self, message: str):
        """Publish status message và log ra terminal."""
        self.get_logger().info(f"[STATUS] {message}")
        m = String(); m.data = message
        self._status_pub.publish(m)


# ════════════════════════════════════════════════════════
#  MAIN
# ════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)

    try:
        node = CheckpointNavigator()
    except RuntimeError as e:
        import logging
        logging.getLogger("navigator_v2").error(
            f"CheckpointNavigator failed to start: {e}")
        rclpy.shutdown()
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
