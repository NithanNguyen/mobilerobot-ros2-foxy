#!/usr/bin/env python3
"""
vel_compare_logger.py
=====================
Compare command velocity (/cmd_vel) against actual velocity (/odom + /odometry/filtered)
to calibrate CMD_SCALE_SPEED and CMD_SCALE_STEER in wheel_odom_node.py.

Topics subscribed:
  /cmd_vel                  — geometry_msgs/Twist  (setpoint from Nav2/DWB)
  /odom                     — nav_msgs/Odometry    (actual velocity from STM32 encoder)
  /odometry/filtered        — nav_msgs/Odometry    (EKF fusion, smoother)

Output:
  vel_compare_<timestamp>.csv   — raw data, one row per snapshot at 10 Hz
  vel_compare_<timestamp>.log   — console log + summary on exit

Usage:
  # Make sure the Nav2 stack is running, then:
  python3 vel_compare_logger.py

  # Or specify an output directory:
  python3 /home/nguyenan/mbrobot_ws/src/mobile_robot/scripts/tools/vel_compare_logger.py --output-dir ~/logs

  # Auto-stop after 60 seconds:
  python3 /home/nguyenan/mbrobot_ws/src/mobile_robot/scripts/tools/vel_compare_logger.py --timeout 60

CSV columns:
  timestamp_sec, cmd_vx, cmd_wz,
  odom_vx, odom_wz,
  filtered_vx, filtered_wz,
  err_vx, err_wz,
  err_vx_pct, err_wz_pct,
  ratio_vx, ratio_wz
"""

import argparse
import csv
import math
import os
import signal
import sys
import threading
import time
from datetime import datetime
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy

# ── Thresholds ────────────────────────────────────────────────────────────────
# Only log when the robot is actually moving (avoids noise while stationary)
MIN_CMD_VX   = 0.01   # m/s   — minimum linear command to log
MIN_CMD_WZ   = 0.02   # rad/s — minimum angular command to log

# Logging rate (Hz)
LOG_RATE_HZ  = 10

# Maximum time to wait for a topic to appear (s)
TOPIC_TIMEOUT = 10.0

# ── Current params from wheel_odom_node.py (printed in summary for reference) ─
CURRENT_CMD_SCALE_SPEED = 116
CURRENT_CMD_SCALE_STEER = 50
CURRENT_STM32_SPEED_MAX = 70
CURRENT_STM32_STEER_MAX = 60
WHEEL_RADIUS             = 0.084
WHEEL_BASE               = 0.464


# ─────────────────────────────────────────────────────────────────────────────
class VelCompareLogger(Node):

    def __init__(self, output_dir: str):
        super().__init__('vel_compare_logger')

        # ── State ──────────────────────────────────────────────────────────
        self._lock = threading.Lock()

        self._cmd:      Optional[Tuple[float, float]] = None   # (vx, wz)
        self._odom:     Optional[Tuple[float, float]] = None   # (vx, wz)
        self._filtered: Optional[Tuple[float, float]] = None   # (vx, wz)

        self._cmd_time:      float = 0.0
        self._odom_time:     float = 0.0
        self._filtered_time: float = 0.0

        # ── Stats accumulators ─────────────────────────────────────────────
        self._n_samples     = 0
        self._sum_err_vx    = 0.0
        self._sum_err_wz    = 0.0
        self._sum_err_vx_sq = 0.0
        self._sum_err_wz_sq = 0.0
        self._sum_ratio_vx  = 0.0
        self._sum_ratio_wz  = 0.0
        self._n_ratio_vx    = 0   # only counted when cmd != 0
        self._n_ratio_wz    = 0

        self._max_err_vx    = 0.0
        self._max_err_wz    = 0.0
        self._start_time    = time.time()

        # ── Output files ───────────────────────────────────────────────────
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        os.makedirs(output_dir, exist_ok=True)

        self._csv_path = os.path.join(output_dir, f'vel_compare_{ts}.csv')
        self._log_path = os.path.join(output_dir, f'vel_compare_{ts}.log')

        self._csv_file = open(self._csv_path, 'w', newline='')
        self._log_file = open(self._log_path, 'w')

        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow([
            'timestamp_sec',
            'cmd_vx',    'cmd_wz',
            'odom_vx',   'odom_wz',
            'filt_vx',   'filt_wz',
            'err_vx',    'err_wz',
            'err_vx_pct','err_wz_pct',
            'ratio_vx',  'ratio_wz',
        ])
        self._csv_file.flush()

        self._log(f'{"="*60}')
        self._log(f'vel_compare_logger — started {datetime.now().isoformat()}')
        self._log(f'CSV  → {self._csv_path}')
        self._log(f'LOG  → {self._log_path}')
        self._log(f'{"="*60}')
        self._log(f'Current wheel_odom_node.py params:')
        self._log(f'  CMD_SCALE_SPEED = {CURRENT_CMD_SCALE_SPEED}')
        self._log(f'  CMD_SCALE_STEER = {CURRENT_CMD_SCALE_STEER}')
        self._log(f'  STM32_SPEED_MAX = {CURRENT_STM32_SPEED_MAX}')
        self._log(f'  STM32_STEER_MAX = {CURRENT_STM32_STEER_MAX}')
        self._log(f'  WHEEL_RADIUS    = {WHEEL_RADIUS} m')
        self._log(f'  WHEEL_BASE      = {WHEEL_BASE} m')
        self._log(f'{"="*60}')
        self._log(f'Subscribing... (press Ctrl+C or wait for timeout to stop and see summary)')
        self._log('')

        # ── QoS ───────────────────────────────────────────────────────────
        qos_sensor = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # ── Subscribers ────────────────────────────────────────────────────
        self.create_subscription(
            Twist, '/cmd_vel',
            self._cb_cmd, 10)

        self.create_subscription(
            Odometry, '/odom',
            self._cb_odom, qos_sensor)

        self.create_subscription(
            Odometry, '/odometry/filtered',
            self._cb_filtered, qos_sensor)

        # ── Log timer ─────────────────────────────────────────────────────
        self._timer = self.create_timer(1.0 / LOG_RATE_HZ, self._log_tick)

        self.get_logger().info(
            f'Logger ready | rate={LOG_RATE_HZ}Hz | '
            f'min_cmd_vx={MIN_CMD_VX} m/s | min_cmd_wz={MIN_CMD_WZ} rad/s')

    # ── Callbacks ─────────────────────────────────────────────────────────────
    def _cb_cmd(self, msg: Twist):
        with self._lock:
            self._cmd      = (msg.linear.x, msg.angular.z)
            self._cmd_time = time.time()

    def _cb_odom(self, msg: Odometry):
        with self._lock:
            self._odom      = (msg.twist.twist.linear.x,
                               msg.twist.twist.angular.z)
            self._odom_time = time.time()

    def _cb_filtered(self, msg: Odometry):
        with self._lock:
            self._filtered      = (msg.twist.twist.linear.x,
                                   msg.twist.twist.angular.z)
            self._filtered_time = time.time()

    # ── Log tick ──────────────────────────────────────────────────────────────
    def _log_tick(self):
        now = time.time()

        with self._lock:
            cmd           = self._cmd
            odom          = self._odom
            filtered      = self._filtered
            cmd_age       = now - self._cmd_time
            odom_age      = now - self._odom_time
            filtered_age  = now - self._filtered_time

        # Skip if no data yet
        if cmd is None or odom is None:
            return

        # Skip if data is stale (> 0.5 s)
        if cmd_age > 0.5 or odom_age > 0.5:
            return

        # Drop filtered if stale (CSV only; does not affect ratio stats)
        if filtered is not None and filtered_age > 0.5:
            filtered = None

        cmd_vx,  cmd_wz  = cmd
        odom_vx, odom_wz = odom
        filt_vx  = filtered[0] if filtered else float('nan')
        filt_wz  = filtered[1] if filtered else float('nan')

        # Only log when the robot is actually moving
        moving = (abs(cmd_vx) >= MIN_CMD_VX or abs(cmd_wz) >= MIN_CMD_WZ)
        if not moving:
            return

        # ── Compute error (odom used as ground truth) ─────────────────────
        err_vx = cmd_vx - odom_vx
        err_wz = cmd_wz - odom_wz

        err_vx_pct = (err_vx / cmd_vx * 100.0) if abs(cmd_vx) > 1e-4 else float('nan')
        err_wz_pct = (err_wz / cmd_wz * 100.0) if abs(cmd_wz) > 1e-4 else float('nan')

        # ratio = odom / cmd — ideal value is 1.0
        ratio_vx = (odom_vx / cmd_vx) if abs(cmd_vx) > 1e-4 else float('nan')
        ratio_wz = (odom_wz / cmd_wz) if abs(cmd_wz) > 1e-4 else float('nan')

        # Check whether the STM32 is hard-clamping the command.
        # When clamped, the computed ratio does not reflect CMD_SCALE, so exclude from stats.
        speed_clamped = abs(cmd_vx) * CURRENT_CMD_SCALE_SPEED > CURRENT_STM32_SPEED_MAX
        steer_clamped = abs(cmd_wz) * CURRENT_CMD_SCALE_STEER > CURRENT_STM32_STEER_MAX

        # ── Accumulate stats ──────────────────────────────────────────────
        self._n_samples     += 1
        self._sum_err_vx    += abs(err_vx)
        self._sum_err_wz    += abs(err_wz)
        self._sum_err_vx_sq += err_vx ** 2
        self._sum_err_wz_sq += err_wz ** 2
        self._max_err_vx     = max(self._max_err_vx, abs(err_vx))
        self._max_err_wz     = max(self._max_err_wz, abs(err_wz))

        if not math.isnan(ratio_vx) and not speed_clamped:
            self._sum_ratio_vx += ratio_vx
            self._n_ratio_vx   += 1
        if not math.isnan(ratio_wz) and not steer_clamped:
            self._sum_ratio_wz += ratio_wz
            self._n_ratio_wz   += 1

        # ── Write CSV ─────────────────────────────────────────────────────
        self._csv_writer.writerow([
            f'{now:.4f}',
            f'{cmd_vx:.4f}',   f'{cmd_wz:.4f}',
            f'{odom_vx:.4f}',  f'{odom_wz:.4f}',
            f'{filt_vx:.4f}',  f'{filt_wz:.4f}',
            f'{err_vx:.4f}',   f'{err_wz:.4f}',
            f'{err_vx_pct:.1f}' if not math.isnan(err_vx_pct) else 'nan',
            f'{err_wz_pct:.1f}' if not math.isnan(err_wz_pct) else 'nan',
            f'{ratio_vx:.4f}'  if not math.isnan(ratio_vx) else 'nan',
            f'{ratio_wz:.4f}'  if not math.isnan(ratio_wz) else 'nan',
        ])
        self._csv_file.flush()

        # ── Console print ─────────────────────────────────────────────────
        line = (
            f'[{self._n_samples:5d}] '
            f'CMD vx={cmd_vx:+.3f} wz={cmd_wz:+.3f} | '
            f'ODOM vx={odom_vx:+.3f} wz={odom_wz:+.3f} | '
            f'ERR vx={err_vx:+.3f}({err_vx_pct:+.1f}%) '
            f'wz={err_wz:+.3f}({err_wz_pct:+.1f}%) | '
            f'RATIO vx={ratio_vx:.3f} wz={ratio_wz:.3f}'
            if not math.isnan(err_vx_pct) and not math.isnan(ratio_vx)
            else
            f'[{self._n_samples:5d}] '
            f'CMD vx={cmd_vx:+.3f} wz={cmd_wz:+.3f} | '
            f'ODOM vx={odom_vx:+.3f} wz={odom_wz:+.3f} | '
            f'ERR vx={err_vx:+.3f} wz={err_wz:+.3f}'
        )
        self._log(line)

    # ── Summary on exit ───────────────────────────────────────────────────────
    def print_summary(self):
        n = self._n_samples
        elapsed = time.time() - self._start_time

        self._log('')
        self._log('=' * 60)
        self._log(f'SUMMARY — {n} samples | elapsed {elapsed:.1f}s')
        self._log('=' * 60)

        if n == 0:
            self._log('No samples recorded (robot never moved?).')
            self._close()
            return

        mae_vx  = self._sum_err_vx / n
        mae_wz  = self._sum_err_wz / n
        rmse_vx = math.sqrt(self._sum_err_vx_sq / n)
        rmse_wz = math.sqrt(self._sum_err_wz_sq / n)

        avg_ratio_vx = (self._sum_ratio_vx / self._n_ratio_vx
                        if self._n_ratio_vx > 0 else float('nan'))
        avg_ratio_wz = (self._sum_ratio_wz / self._n_ratio_wz
                        if self._n_ratio_wz > 0 else float('nan'))

        self._log(f'  Linear  vx — MAE={mae_vx:.4f} m/s | RMSE={rmse_vx:.4f} | '
                  f'MAX_ERR={self._max_err_vx:.4f} | avg_ratio={avg_ratio_vx:.4f}')
        self._log(f'  Angular wz — MAE={mae_wz:.4f} rad/s | RMSE={rmse_wz:.4f} | '
                  f'MAX_ERR={self._max_err_wz:.4f} | avg_ratio={avg_ratio_wz:.4f}')

        self._log('')
        self._log('ANALYSIS — Suggested adjustments for wheel_odom_node.py:')
        self._log('-' * 60)

        # ── Analyse ratio_vx ──────────────────────────────────────────────
        if not math.isnan(avg_ratio_vx):
            if 0.95 <= avg_ratio_vx <= 1.05:
                self._log(f'  [OK]  vx ratio={avg_ratio_vx:.3f} — CMD_SCALE_SPEED looks good.')
            else:
                new_scale = round(CURRENT_CMD_SCALE_SPEED / avg_ratio_vx)
                self._log(f'  [FIX] vx ratio={avg_ratio_vx:.3f} (ideal=1.000)')
                self._log(f'        → Robot travels at {avg_ratio_vx*100:.1f}% of the command.')
                if avg_ratio_vx < 1.0:
                    self._log(f'        → Robot is SLOWER than commanded → increase CMD_SCALE_SPEED')
                else:
                    self._log(f'        → Robot is FASTER than commanded → decrease CMD_SCALE_SPEED')
                self._log(f'        Suggestion: CMD_SCALE_SPEED = {new_scale} '
                          f'(current = {CURRENT_CMD_SCALE_SPEED})')

        # ── Analyse ratio_wz ──────────────────────────────────────────────
        if not math.isnan(avg_ratio_wz):
            if 0.95 <= avg_ratio_wz <= 1.05:
                self._log(f'  [OK]  wz ratio={avg_ratio_wz:.3f} — CMD_SCALE_STEER looks good.')
            else:
                new_scale = round(CURRENT_CMD_SCALE_STEER / avg_ratio_wz)
                self._log(f'  [FIX] wz ratio={avg_ratio_wz:.3f} (ideal=1.000)')
                if avg_ratio_wz < 1.0:
                    self._log(f'        → Robot turns SLOWER than commanded → increase CMD_SCALE_STEER')
                else:
                    self._log(f'        → Robot turns FASTER than commanded → decrease CMD_SCALE_STEER')
                self._log(f'        Suggestion: CMD_SCALE_STEER = {new_scale} '
                          f'(current = {CURRENT_CMD_SCALE_STEER})')

        # ── Method limitations ────────────────────────────────────────────
        self._log('')
        self._log('IMPORTANT NOTES:')
        self._log('  - Ratio is derived from /odom — accurate only if WHEEL_RADIUS and WHEEL_BASE are correct.')
        self._log('  - If the robot moves correctly but /odom is wrong →')
        self._log('    calibrate WHEEL_RADIUS/WHEEL_BASE first (measure physically).')
        self._log('  - Test at multiple speed levels to avoid bias.')
        self._log(f'  - Full CSV: {self._csv_path}')
        self._log('=' * 60)

        self._close()

    # ── Helpers ───────────────────────────────────────────────────────────────
    def _log(self, msg: str):
        print(msg, flush=True)
        self._log_file.write(msg + '\n')
        self._log_file.flush()

    def _close(self):
        try:
            self._csv_file.close()
        except Exception:
            pass
        try:
            self._log_file.close()
        except Exception:
            pass


# ─────────────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(
        description='Compare /cmd_vel vs /odom to calibrate wheel_odom_node.py')
    parser.add_argument(
        '--output-dir', '-o',
        default=os.path.expanduser('~/mbrobot_ws/src/mobile_robot/logs'),
        help='Directory for CSV and LOG files (default: ~/mbrobot_ws/src/mobile_robot/logs)')
    parser.add_argument(
        '--timeout', '-t',
        type=float, default=None,
        metavar='SECONDS',
        help='Auto-stop after N seconds (default: run until Ctrl+C)')
    args = parser.parse_args()

    rclpy.init()
    node = VelCompareLogger(output_dir=args.output_dir)

    # Ensure shutdown runs exactly once regardless of which trigger fires first
    _done = threading.Event()

    def _do_shutdown(reason: str):
        if _done.is_set():
            return
        _done.set()
        print(f'\n[{reason}] Stopping and computing summary...')
        node.print_summary()
        try:
            rclpy.shutdown()
        except Exception:
            pass

    signal.signal(signal.SIGINT, lambda sig, frame: _do_shutdown('SIGINT'))

    if args.timeout is not None:
        def _timeout_worker():
            time.sleep(args.timeout)
            _do_shutdown(f'TIMEOUT {args.timeout:.0f}s')
            # sys.exit() from a non-main thread does not kill the main thread;
            # send SIGINT so rclpy.spin() on the main thread unblocks.
            os.kill(os.getpid(), signal.SIGINT)
        threading.Thread(target=_timeout_worker, daemon=True).start()
        print(f'Running... Auto-stop in {args.timeout:.0f}s or press Ctrl+C.\n')
    else:
        print('Running... Press Ctrl+C to stop and view summary.\n')

    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
