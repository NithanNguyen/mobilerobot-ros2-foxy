#!/usr/bin/env python3
"""
endurance_runner.py
====================
Autonomous loop navigation node for real-world endurance testing.
Drives the robot along a fixed route (loop) and logs per-leg timing
to CSV — from battery-full to battery-dead.

Control:
  /endurance/start  (std_msgs/Bool True)  → begin loop, reset session timer
  /endurance/stop   (std_msgs/Bool True)  → cancel goal immediately, stop logging
  /robot/emergency_stop (std_msgs/Bool)   → pause/resume without consuming a retry

Route:  ROS parameter  route:="[0,1,2,3]"   (checkpoint IDs, loops indefinitely)
Log:    ~/endurance_logs/endurance_YYYYMMDD_HHMMSS.csv  (flushed every leg)
"""

import csv
import math
import os
import time
import yaml
from datetime import datetime
from enum import Enum
from pathlib import Path as FsPath
from typing import List, Optional

import rclpy
import rclpy.time
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Bool, String


# ── Retry / timing constants ────────────────────────────────────────────────
MAX_RETRIES      = 3
RETRY_DELAY_S    = 5.0
NAV_POLL_HZ      = 5.0       # state-machine tick rate


# ── State machine ────────────────────────────────────────────────────────────
class RunnerState(Enum):
    IDLE           = "IDLE"
    NAVIGATING     = "NAVIGATING"
    RETRY_WAIT     = "RETRY_WAIT"
    EMERGENCY_STOP = "EMERGENCY_STOP"
    STOPPED        = "STOPPED"


# ════════════════════════════════════════════════════════════════════════════
class EnduranceRunner(Node):

    def __init__(self):
        super().__init__("endurance_runner")

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter("checkpoint_file", "")
        self.declare_parameter("route", [0, 1, 2, 3, 4])
        self.declare_parameter("goal_tolerance", 0.25)

        self._route: List[int] = list(
            self.get_parameter("route").get_parameter_value().integer_array_value
        )
        self._goal_tolerance: float = (
            self.get_parameter("goal_tolerance").value
        )

        # ── Load checkpoints ─────────────────────────────────────────────────
        self._checkpoints: dict = self._load_checkpoints()
        if not self._checkpoints:
            self.get_logger().error("No checkpoints loaded — aborting.")
            return
        self._validate_route()

        # ── Runtime state ────────────────────────────────────────────────────
        self._state            = RunnerState.IDLE
        self._route_idx        = 0          # index into self._route
        self._lap              = 0          # full loops completed
        self._retry_count      = 0
        self._retry_wait_start = 0.0

        self._session_start:   Optional[float] = None
        self._leg_start:       Optional[float] = None
        self._leg_estop_count: int = 0
        self._estop_active:    bool = False
        self._state_before_estop = RunnerState.IDLE

        self._goal_handle = None

        # ── CSV log ──────────────────────────────────────────────────────────
        self._csv_file   = None
        self._csv_writer = None
        self._log_path:  Optional[str] = None

        # ── Nav2 action client ───────────────────────────────────────────────
        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.get_logger().info("Waiting for navigate_to_pose action server...")
        self._nav_client.wait_for_server()
        self.get_logger().info("Action server ready.")

        # ── Publishers ───────────────────────────────────────────────────────
        self._status_pub = self.create_publisher(String, "/endurance/status", 10)

        # ── Subscribers ──────────────────────────────────────────────────────
        self.create_subscription(Bool, "/endurance/start",      self._on_start,  10)
        self.create_subscription(Bool, "/endurance/stop",       self._on_stop,   10)
        self.create_subscription(Bool, "/robot/emergency_stop", self._on_estop,  10)

        # ── Main loop timer ──────────────────────────────────────────────────
        self.create_timer(1.0 / NAV_POLL_HZ, self._tick)

        self.get_logger().info(
            f"EnduranceRunner ready. Route: {self._route}  "
            f"Max retries: {MAX_RETRIES}  "
            f"Retry delay: {RETRY_DELAY_S}s\n"
            f"Publish True to /endurance/start to begin."
        )

    # ════════════════════════════════════════════════════════════════════════
    #  CHECKPOINT LOADING
    # ════════════════════════════════════════════════════════════════════════
    def _load_checkpoints(self) -> dict:
        path = self.get_parameter("checkpoint_file").value
        if not path or not os.path.exists(path):
            try:
                pkg  = get_package_share_directory("mobile_robot")
                path = os.path.join(pkg, "config", "checkpoints.yaml")
            except Exception:
                self.get_logger().error("Package 'mobile_robot' not found.")
                return {}

        if not os.path.exists(path):
            self.get_logger().error(f"Checkpoint file not found: {path}")
            return {}

        with open(path, "r") as f:
            data = yaml.safe_load(f)

        result = {}
        frame  = data.get("frame_id", "map")
        for cp in data.get("checkpoints", []):
            pose                    = PoseStamped()
            pose.header.frame_id    = frame
            pose.pose.position.x    = float(cp["position"]["x"])
            pose.pose.position.y    = float(cp["position"]["y"])
            pose.pose.position.z    = float(cp["position"].get("z", 0.0))
            pose.pose.orientation.x = float(cp["orientation"].get("x", 0.0))
            pose.pose.orientation.y = float(cp["orientation"].get("y", 0.0))
            pose.pose.orientation.z = float(cp["orientation"].get("z", 0.0))
            pose.pose.orientation.w = float(cp["orientation"].get("w", 1.0))
            result[cp["id"]] = {
                "id":   cp["id"],
                "name": cp.get("display_name", cp.get("name", str(cp["id"]))),
                "pose": pose,
            }
        self.get_logger().info(f"Loaded {len(result)} checkpoints from {path}")
        return result

    def _validate_route(self):
        invalid = [r for r in self._route if r not in self._checkpoints]
        if invalid:
            self.get_logger().error(
                f"Route contains unknown checkpoint IDs: {invalid}. "
                f"Valid: {list(self._checkpoints.keys())}"
            )
            raise ValueError(f"Invalid route checkpoint IDs: {invalid}")

    # ════════════════════════════════════════════════════════════════════════
    #  CONTROL CALLBACKS
    # ════════════════════════════════════════════════════════════════════════
    def _on_start(self, msg: Bool):
        if not msg.data:
            return
        if self._state not in (RunnerState.IDLE, RunnerState.STOPPED):
            self.get_logger().warn(
                f"Start ignored — currently in state [{self._state.name}].")
            return

        self.get_logger().info("=== ENDURANCE TEST STARTED ===")
        self._session_start   = time.time()
        self._route_idx       = 0
        self._lap             = 0
        self._retry_count     = 0
        self._leg_estop_count = 0
        self._open_log()
        self._state = RunnerState.NAVIGATING
        self._send_next_goal()

    def _on_stop(self, msg: Bool):
        if not msg.data:
            return
        if self._state == RunnerState.IDLE:
            return

        self.get_logger().info("=== ENDURANCE TEST STOPPED ===")
        self._cancel_current_goal()
        self._state = RunnerState.STOPPED
        self._close_log()
        self._pub_status("STOPPED by operator.")

    def _on_estop(self, msg: Bool):
        if msg.data:
            # ── Activate emergency stop ──────────────────────────────────────
            if not self._estop_active:
                self._estop_active       = True
                self._state_before_estop = self._state
                self._leg_estop_count   += 1
                self.get_logger().warn(
                    "[ESTOP] Emergency stop received — pausing endurance test.")
                self._cancel_current_goal()
                if self._state not in (RunnerState.IDLE, RunnerState.STOPPED):
                    self._state = RunnerState.EMERGENCY_STOP
                self._pub_status("EMERGENCY STOP — waiting for reset.")
        else:
            # ── Release emergency stop ───────────────────────────────────────
            if self._estop_active:
                self._estop_active = False
                self.get_logger().info(
                    "[ESTOP] Emergency stop cleared — resuming.")
                if self._state_before_estop == RunnerState.NAVIGATING:
                    # Resume the same leg (no retry consumed)
                    self._state = RunnerState.NAVIGATING
                    self._send_next_goal()
                else:
                    self._state = self._state_before_estop
                self._pub_status("Resumed after ESTOP.")

    # ════════════════════════════════════════════════════════════════════════
    #  STATE MACHINE TICK
    # ════════════════════════════════════════════════════════════════════════
    def _tick(self):
        if self._state == RunnerState.RETRY_WAIT:
            if time.time() - self._retry_wait_start >= RETRY_DELAY_S:
                self.get_logger().info(
                    f"[RETRY] Retry {self._retry_count}/{MAX_RETRIES} "
                    f"for CP [{self._current_cp_id()}]")
                self._state = RunnerState.NAVIGATING
                self._send_next_goal()

    # ════════════════════════════════════════════════════════════════════════
    #  NAVIGATION
    # ════════════════════════════════════════════════════════════════════════
    def _current_cp_id(self) -> int:
        return self._route[self._route_idx]

    def _send_next_goal(self):
        cp_id = self._current_cp_id()
        cp    = self._checkpoints[cp_id]

        pose              = cp["pose"]
        pose.header.stamp = self.get_clock().now().to_msg()

        goal      = NavigateToPose.Goal()
        goal.pose = pose

        self._leg_start = time.time()

        self.get_logger().info(
            f"[NAV] Lap {self._lap} | "
            f"Leg {self._route_idx + 1}/{len(self._route)} | "
            f"→ [{cp_id}] {cp['name']}  "
            f"(retry={self._retry_count})"
        )
        self._pub_status(
            f"Lap {self._lap} | Navigating → [{cp_id}] {cp['name']}"
        )

        send_future = self._nav_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_accepted)

    def _on_goal_accepted(self, future):
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().error("Goal REJECTED by Nav2.")
            self._handle_leg_failure("REJECTED")
            return
        self._goal_handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        # Guard: if stopped or estop triggered while waiting, ignore stale result
        if self._state in (RunnerState.STOPPED, RunnerState.EMERGENCY_STOP):
            return

        status = future.result().status
        cp_id  = self._current_cp_id()

        if status == GoalStatus.STATUS_SUCCEEDED:
            self._handle_leg_success(cp_id)
        elif status == GoalStatus.STATUS_CANCELED:
            # Cancellation is intentional (stop/estop) — don't treat as failure
            self.get_logger().info(f"Goal to [{cp_id}] canceled.")
        else:
            self._handle_leg_failure("ABORTED")

    # ── Success path ─────────────────────────────────────────────────────────
    def _handle_leg_success(self, cp_id: int):
        travel_time   = time.time() - self._leg_start
        total_runtime = time.time() - self._session_start
        cp_name       = self._checkpoints[cp_id]["name"]

        self.get_logger().info(
            f"[ARRIVED] [{cp_id}] {cp_name} | "
            f"Travel: {travel_time:.1f}s | "
            f"Total runtime: {self._fmt_duration(total_runtime)} | "
            f"EStops this leg: {self._leg_estop_count}"
        )

        # ── Write CSV row ────────────────────────────────────────────────────
        from_idx  = (self._route_idx - 1) % len(self._route)
        from_cp   = self._route[from_idx]
        self._write_log_row(
            lap          = self._lap,
            leg          = f"{from_cp}→{cp_id}",
            cp_name      = cp_name,
            travel_time  = round(travel_time, 2),
            status       = "SUCCESS",
            retries_used = self._retry_count,
            estop_count  = self._leg_estop_count,
            total_runtime= round(total_runtime, 2),
        )

        # ── Advance route ────────────────────────────────────────────────────
        self._retry_count     = 0
        self._leg_estop_count = 0
        self._route_idx      += 1

        if self._route_idx >= len(self._route):
            self._route_idx = 0
            self._lap      += 1
            self.get_logger().info(
                f"[LAP] Lap {self._lap - 1} complete. "
                f"Total runtime: {self._fmt_duration(total_runtime)}"
            )

        # ── Continue loop if still running ──────────────────────────────────
        if self._state == RunnerState.NAVIGATING:
            self._send_next_goal()

    # ── Failure / retry path ─────────────────────────────────────────────────
    def _handle_leg_failure(self, reason: str):
        cp_id         = self._current_cp_id()
        travel_time   = time.time() - self._leg_start
        total_runtime = time.time() - self._session_start

        self.get_logger().error(
            f"[FAIL] [{cp_id}] status={reason} | "
            f"Retry {self._retry_count}/{MAX_RETRIES}"
        )

        if self._retry_count < MAX_RETRIES:
            self._retry_count     += 1
            self._state            = RunnerState.RETRY_WAIT
            self._retry_wait_start = time.time()
            self._pub_status(
                f"Leg to [{cp_id}] {reason}. "
                f"Retry {self._retry_count}/{MAX_RETRIES} in {RETRY_DELAY_S:.0f}s..."
            )
        else:
            # Max retries exhausted → log ABORTED and skip to next checkpoint
            from_idx = (self._route_idx - 1) % len(self._route)
            from_cp  = self._route[from_idx]
            self._write_log_row(
                lap          = self._lap,
                leg          = f"{from_cp}→{cp_id}",
                cp_name      = self._checkpoints[cp_id]["name"],
                travel_time  = round(travel_time, 2),
                status       = f"ABORTED_SKIP ({reason})",
                retries_used = self._retry_count,
                estop_count  = self._leg_estop_count,
                total_runtime= round(total_runtime, 2),
            )
            self.get_logger().error(
                f"[SKIP] Max retries reached for [{cp_id}]. Skipping to next checkpoint."
            )
            self._retry_count     = 0
            self._leg_estop_count = 0
            self._route_idx      += 1
            if self._route_idx >= len(self._route):
                self._route_idx = 0
                self._lap      += 1
            if self._state == RunnerState.NAVIGATING:
                self._send_next_goal()

    def _cancel_current_goal(self):
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None

    # ════════════════════════════════════════════════════════════════════════
    #  CSV LOGGING
    # ════════════════════════════════════════════════════════════════════════
    CSV_FIELDS = [
        "timestamp", "lap", "leg", "checkpoint_name",
        "travel_time_s", "status", "retries_used",
        "estop_count_leg", "total_runtime_s", "total_runtime_hms",
    ]

    def _open_log(self):
        log_dir = FsPath.home() / "endurance_logs"
        log_dir.mkdir(parents=True, exist_ok=True)

        stamp         = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._log_path = str(log_dir / f"endurance_{stamp}.csv")

        self._csv_file   = open(self._log_path, "w", newline="")
        self._csv_writer = csv.DictWriter(
            self._csv_file, fieldnames=self.CSV_FIELDS)
        self._csv_writer.writeheader()
        self._csv_file.flush()

        self.get_logger().info(f"[LOG] Logging to: {self._log_path}")

    def _write_log_row(self, lap, leg, cp_name, travel_time,
                       status, retries_used, estop_count, total_runtime):
        if self._csv_writer is None:
            return
        self._csv_writer.writerow({
            "timestamp":        datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "lap":              lap,
            "leg":              leg,
            "checkpoint_name":  cp_name,
            "travel_time_s":    travel_time,
            "status":           status,
            "retries_used":     retries_used,
            "estop_count_leg":  estop_count,
            "total_runtime_s":  total_runtime,
            "total_runtime_hms":self._fmt_duration(total_runtime),
        })
        self._csv_file.flush()   # write to disk immediately

    def _close_log(self):
        if self._csv_file:
            total = time.time() - self._session_start if self._session_start else 0
            self.get_logger().info(
                f"[LOG] Session ended. "
                f"Total runtime: {self._fmt_duration(total)} | "
                f"Laps completed: {self._lap} | "
                f"File: {self._log_path}"
            )
            self._csv_file.close()
            self._csv_file   = None
            self._csv_writer = None

    # ════════════════════════════════════════════════════════════════════════
    #  UTILITIES
    # ════════════════════════════════════════════════════════════════════════
    @staticmethod
    def _fmt_duration(seconds: float) -> str:
        h = int(seconds // 3600)
        m = int((seconds % 3600) // 60)
        s = int(seconds % 60)
        return f"{h:02d}:{m:02d}:{s:02d}"

    def _pub_status(self, message: str):
        msg      = String()
        msg.data = message
        self._status_pub.publish(msg)


# ════════════════════════════════════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)
    node = EnduranceRunner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._close_log()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
