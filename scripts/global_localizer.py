#!/usr/bin/env python3
"""
global_localizer.py
==========================================
Autonomous global localization at startup — removes the need for a hardcoded
initial pose in the launch file / nav2 params.

Replaces the old `set_initial_pose` ExecuteProcess step in nav_v2.launch.py.

Sequence (runs once in a worker thread, then the node idles):

  STEP 1  Wait for AMCL to be fully active
  STEP 2  Call /reinitialize_global_localization (spread particles over the map)
  STEP 3  Spin in place (publish /cmd_vel) to gather LiDAR observations
  STEP 4  After each full 360° rotation, check AMCL pose covariance convergence
  STEP 5  Stop, confirm the converged pose back to AMCL via /initialpose
  STEP 6  Publish std_msgs/Bool True on /global_localizer/ready, then idle

Design notes:
  - NO Nav2 action clients are used (lightweight, no action-server timing races).
    The spin is an open-loop /cmd_vel command — the Nav2 Spin behavior is avoided
    because the costmap is not reliable before localization converges.
  - The /cmd_vel spin goes through the same topic that wheel_odom_node consumes.
    There is NO hard-stop velocity gate on /cmd_vel in this stack (ultrasonic
    fusion only feeds the costmap), and 0.4 rad/s is far below wheel_odom_node's
    MAX_ANGULAR_VEL clamp (1.5 rad/s), so the spin is safe.
  - The procedure runs in a background thread while a MultiThreadedExecutor spins
    the node so subscriptions and the service future are serviced concurrently.

Topics / Services:
  Sub:  /amcl_pose          (geometry_msgs/PoseWithCovarianceStamped)
  Pub:  /cmd_vel            (geometry_msgs/Twist)            — spin command
  Pub:  /initialpose        (geometry_msgs/PoseWithCovarianceStamped) — confirm pose
  Pub:  /global_localizer/ready (std_msgs/Bool, latched)    — readiness signal
  Srv:  /reinitialize_global_localization (std_srvs/Empty)  — client

Parameters (tunable from nav2_params_test.yaml):
  convergence_threshold_xy   0.10  (float, m²)   — σ²x and σ²y ceiling
  convergence_threshold_yaw  0.15  (float, rad²) — σ²yaw ceiling
  spin_angular_velocity      0.4   (float, rad/s)
  max_spin_attempts          3     (int)
  amcl_wait_timeout          30.0  (float, s)
"""

import math
import threading
import time
from typing import Optional

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from std_msgs.msg import Bool
from std_srvs.srv import Empty


# ============================================================
#  TUNING PARAMETERS (defaults — override via ROS params)
# ============================================================

# Service AMCL exposes to scatter all particles uniformly over the map free space
REINIT_SERVICE      = "/reinitialize_global_localization"

# How often to publish the spin Twist while rotating (10 Hz)
SPIN_PUBLISH_PERIOD = 0.1     # s

# Settle time after a rotation before reading covariance — lets AMCL run a couple
# of filter updates on the freshly observed scans.
SETTLE_TIME         = 1.5     # s

# Indices into the 36-element flat covariance matrix (row-major 6x6)
COV_IDX_X   = 0      # σ²x
COV_IDX_Y   = 7      # σ²y
COV_IDX_YAW = 35     # σ²yaw

# ============================================================


class GlobalLocalizer(Node):
    """
    Performs one-shot autonomous global localization at startup, then idles.
    """

    def __init__(self):
        super().__init__("global_localizer")

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter("convergence_threshold_xy",  0.10)
        self.declare_parameter("convergence_threshold_yaw", 0.15)
        self.declare_parameter("spin_angular_velocity",     0.4)
        self.declare_parameter("max_spin_attempts",         3)
        self.declare_parameter("amcl_wait_timeout",         30.0)

        self.thr_xy        = float(self.get_parameter("convergence_threshold_xy").value)
        self.thr_yaw       = float(self.get_parameter("convergence_threshold_yaw").value)
        self.spin_w        = float(self.get_parameter("spin_angular_velocity").value)
        self.max_attempts  = int(self.get_parameter("max_spin_attempts").value)
        self.amcl_timeout  = float(self.get_parameter("amcl_wait_timeout").value)

        # Time for one full rotation at the configured angular velocity (~16 s @ 0.4)
        self.spin_duration = (2.0 * math.pi) / self.spin_w if self.spin_w > 0.0 else 16.0

        # ── State ───────────────────────────────────────────────────────────
        self._pose_lock        = threading.Lock()
        self._last_pose: Optional[PoseWithCovarianceStamped] = None

        # ── Callback group (let subscription + service future run concurrently
        #    with the worker thread under the MultiThreadedExecutor) ──────────
        cb_group = ReentrantCallbackGroup()

        # ── Subscriber: AMCL pose (covariance source for convergence check) ──
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self._on_amcl_pose,
            10,
            callback_group=cb_group,
        )

        # ── Publishers ───────────────────────────────────────────────────────
        self._cmdvel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10)

        # Latched (transient-local) QoS so a late subscriber (e.g. a future
        # version of checkpoint_navigator) still receives the readiness signal.
        ready_qos = QoSProfile(depth=1)
        ready_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self._ready_pub = self.create_publisher(
            Bool, "/global_localizer/ready", ready_qos)

        # ── Service client: global localization ──────────────────────────────
        self._reinit_client = self.create_client(
            Empty, REINIT_SERVICE, callback_group=cb_group)

        self.get_logger().info(
            f"GlobalLocalizer ready │ "
            f"thr_xy={self.thr_xy:.3f} m² │ thr_yaw={self.thr_yaw:.3f} rad² │ "
            f"spin_w={self.spin_w:.2f} rad/s │ "
            f"spin_duration={self.spin_duration:.1f} s │ "
            f"max_attempts={self.max_attempts} │ "
            f"amcl_timeout={self.amcl_timeout:.0f} s")

        # ── Launch the procedure in a background thread ──────────────────────
        # The MultiThreadedExecutor (created in main) spins the node so that the
        # /amcl_pose subscription and the service future are serviced while this
        # worker blocks on time.sleep()/future polling.
        self._worker = threading.Thread(target=self._run_sequence, daemon=True)
        self._worker.start()

    # ════════════════════════════════════════════════════════
    #  SUBSCRIBER CALLBACK
    # ════════════════════════════════════════════════════════

    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped):
        """Store the most recent AMCL pose for the convergence check."""
        with self._pose_lock:
            self._last_pose = msg

    def _get_last_pose(self) -> Optional[PoseWithCovarianceStamped]:
        with self._pose_lock:
            return self._last_pose

    # ════════════════════════════════════════════════════════
    #  MAIN PROCEDURE (worker thread)
    # ════════════════════════════════════════════════════════

    def _run_sequence(self):
        """Run STEP 1..6 once, then leave the node idling."""
        try:
            # ── STEP 1 — wait for AMCL to be fully active ───────────────────
            if not self._wait_for_amcl_active():
                self.get_logger().error(
                    "AMCL did not become active within "
                    f"{self.amcl_timeout:.0f} s. Aborting global localization.")
                return

            # ── STEP 2 — global localization service call ───────────────────
            if not self._call_global_localization():
                self.get_logger().error(
                    "Global localization service call failed. Aborting.")
                return

            # Record covariance BEFORE spinning (acceptance criterion #4).
            cov_before = self._read_covariance()
            if cov_before is not None:
                self.get_logger().info(
                    f"[BEFORE SPIN] σ²x={cov_before[0]:.3f} "
                    f"σ²y={cov_before[1]:.3f} σ²yaw={cov_before[2]:.3f}")

            # ── STEP 3 & 4 — spin in place, check convergence each rotation ──
            converged = self._spin_until_converged()

            # ── STEP 4 (timeout branch) ─────────────────────────────────────
            if not converged:
                self.get_logger().warn(
                    f"Localization NOT converged after {self.max_attempts} "
                    "rotations. Proceeding with best available pose anyway "
                    "(not blocking navigation).")

            # ── STEP 5 — stop and confirm the pose to AMCL ──────────────────
            self._stop_robot()
            self._confirm_pose()

            cov_after = self._read_covariance()
            if cov_after is not None:
                self.get_logger().info(
                    f"[AFTER SPIN] σ²x={cov_after[0]:.3f} "
                    f"σ²y={cov_after[1]:.3f} σ²yaw={cov_after[2]:.3f}")

            # ── STEP 6 — signal readiness ───────────────────────────────────
            self._publish_ready()
            self.get_logger().info(
                "Global localization sequence complete. Node now idle.")

        except Exception as e:  # never let the worker die silently
            self.get_logger().error(f"Global localization sequence crashed: {e}")
            self._stop_robot()

    # ════════════════════════════════════════════════════════
    #  STEP 1 — wait for AMCL active
    # ════════════════════════════════════════════════════════

    def _wait_for_amcl_active(self) -> bool:
        """
        Wait until AMCL is fully active, checking every 1 s up to amcl_wait_timeout.

        The concrete readiness signal is the availability of AMCL's
        /reinitialize_global_localization service: AMCL only advertises it once
        the node has been activated by the Nav2 lifecycle manager. (The /amcl_pose
        topic itself is not published until AFTER global init + the first scan,
        so it cannot be used as a pre-init gate.)
        """
        self.get_logger().info("[STEP 1] Waiting for AMCL to become active...")
        deadline = time.monotonic() + self.amcl_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if self._reinit_client.service_is_ready():
                self.get_logger().info(
                    "[STEP 1] AMCL is active "
                    f"({REINIT_SERVICE} available). ✓")
                return True
            # Also accept a live /amcl_pose as evidence AMCL is up and publishing.
            if self._get_last_pose() is not None:
                self.get_logger().info(
                    "[STEP 1] AMCL is active (/amcl_pose publishing). ✓")
                return True
            self.get_logger().info(
                "[STEP 1] AMCL not ready yet, retrying in 1 s...")
            time.sleep(1.0)
        return False

    # ════════════════════════════════════════════════════════
    #  STEP 2 — global localization service call
    # ════════════════════════════════════════════════════════

    def _call_global_localization(self) -> bool:
        """Call /reinitialize_global_localization to scatter particles."""
        self.get_logger().info(
            f"[STEP 2] Calling {REINIT_SERVICE} ...")
        if not self._reinit_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                f"[STEP 2] {REINIT_SERVICE} unavailable after 5 s.")
            return False

        future = self._reinit_client.call_async(Empty.Request())
        if not self._wait_future(future, timeout=10.0):
            self.get_logger().error(
                "[STEP 2] Global localization service call timed out.")
            return False

        self.get_logger().info(
            "Global localization initialized — particles spread across map")
        return True

    # ════════════════════════════════════════════════════════
    #  STEP 3 & 4 — spin + convergence check
    # ════════════════════════════════════════════════════════

    def _spin_until_converged(self) -> bool:
        """
        Rotate a full 360° up to max_spin_attempts times; after each rotation,
        let AMCL settle and check the pose covariance. Returns True on convergence.
        """
        for attempt in range(1, self.max_attempts + 1):
            self.get_logger().info(
                f"[STEP 3] Rotation attempt {attempt}/{self.max_attempts} — "
                f"spinning 360° at {self.spin_w:.2f} rad/s "
                f"(~{self.spin_duration:.1f} s)...")
            self._spin_one_rotation()

            # Stop and let AMCL run a few filter updates on the new observations.
            self._stop_robot()
            time.sleep(SETTLE_TIME)

            # ── STEP 4 — convergence check ──────────────────────────────────
            cov = self._read_covariance()
            if cov is None:
                self.get_logger().warn(
                    "[STEP 4] No /amcl_pose received yet — cannot check "
                    "convergence this round.")
                continue

            sx, sy, syaw = cov
            converged = (sx < self.thr_xy and sy < self.thr_xy and syaw < self.thr_yaw)
            self.get_logger().info(
                f"[STEP 4] Covariance after rotation {attempt}: "
                f"σ²x={sx:.3f} (<{self.thr_xy}) │ "
                f"σ²y={sy:.3f} (<{self.thr_xy}) │ "
                f"σ²yaw={syaw:.3f} (<{self.thr_yaw}) │ "
                f"converged={converged}")

            if converged:
                self.get_logger().info(
                    f"[STEP 4] ✓ Converged after {attempt} rotation(s).")
                return True

        return False

    def _spin_one_rotation(self):
        """Publish the spin Twist at SPIN_PUBLISH_PERIOD for one full rotation."""
        twist = Twist()
        twist.linear.x  = 0.0
        twist.angular.z = self.spin_w

        start = time.monotonic()
        while rclpy.ok() and (time.monotonic() - start) < self.spin_duration:
            self._cmdvel_pub.publish(twist)
            time.sleep(SPIN_PUBLISH_PERIOD)

    # ════════════════════════════════════════════════════════
    #  STEP 5 — confirm pose to AMCL
    # ════════════════════════════════════════════════════════

    def _confirm_pose(self):
        """
        Re-publish the converged (or best available) pose to /initialpose to
        reinforce AMCL's belief and tighten the particle cloud.
        """
        pose = self._get_last_pose()
        if pose is None:
            self.get_logger().warn(
                "[STEP 5] No AMCL pose available — skipping pose confirmation.")
            return

        out = PoseWithCovarianceStamped()
        out.header.frame_id = pose.header.frame_id or "map"
        out.header.stamp    = self.get_clock().now().to_msg()
        out.pose            = pose.pose  # carries pose + current covariance

        self._initialpose_pub.publish(out)

        x = pose.pose.pose.position.x
        y = pose.pose.pose.position.y
        self.get_logger().info(
            f"Localization converged — pose confirmed at x={x:.3f} y={y:.3f}")

    # ════════════════════════════════════════════════════════
    #  STEP 6 — signal readiness
    # ════════════════════════════════════════════════════════

    def _publish_ready(self):
        """Latch a True on /global_localizer/ready for downstream consumers."""
        msg = Bool()
        msg.data = True
        self._ready_pub.publish(msg)
        self.get_logger().info(
            "[STEP 6] Published True on /global_localizer/ready.")

    # ════════════════════════════════════════════════════════
    #  UTILITIES
    # ════════════════════════════════════════════════════════

    def _read_covariance(self) -> Optional[tuple]:
        """Return (σ²x, σ²y, σ²yaw) from the latest /amcl_pose, or None."""
        pose = self._get_last_pose()
        if pose is None:
            return None
        cov = pose.pose.covariance
        return (cov[COV_IDX_X], cov[COV_IDX_Y], cov[COV_IDX_YAW])

    def _stop_robot(self):
        """Publish a zero Twist to halt the spin."""
        self._cmdvel_pub.publish(Twist())

    def _wait_future(self, future, timeout: float) -> bool:
        """
        Poll a future to completion (the MultiThreadedExecutor running in the
        main thread services the callback that resolves it).
        """
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not future.done():
            if time.monotonic() > deadline:
                return False
            time.sleep(0.05)
        return future.done()


# ════════════════════════════════════════════════════════
#  MAIN
# ════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)

    node = GlobalLocalizer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
