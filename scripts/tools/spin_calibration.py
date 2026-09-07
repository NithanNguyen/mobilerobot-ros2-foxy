#!/usr/bin/env python3
"""
spin_calibration.py
===================
Xoay robot tại chỗ với cmd_wz cố định, đo tỉ lệ odom_wz/cmd_wz để kiểm tra
CMD_SCALE_STEER trong wheel_odom_node.py.

Chạy SAU KHI nav_v2.launch.py đã khởi động:
  python3 /home/nguyenan/mbrobot_ws/src/mobile_robot/scripts/tools/spin_calibration.py

Tuỳ chọn:
  --wz      Vận tốc góc (rad/s), mặc định 0.4
  --duration  Thời gian xoay (s), mặc định 10
  --warmup  Thời gian bỏ qua lúc mới bắt đầu (s), mặc định 2
  --dir     Chiều xoay: cw (ngược chiều kim đồng hồ) hoặc ccw, mặc định ccw
"""

import argparse
import math
import signal
import statistics
import sys
import threading
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

# ── Tham số hiện tại trong wheel_odom_node.py ────────────────────────────────
CURRENT_CMD_SCALE_STEER = 50


# ─────────────────────────────────────────────────────────────────────────────
class SpinCalibrator(Node):

    def __init__(self, cmd_wz: float, duration: float, warmup: float):
        super().__init__('spin_calibrator')

        self._cmd_wz   = cmd_wz
        self._duration = duration
        self._warmup   = warmup

        self._lock        = threading.Lock()
        self._odom_wz: Optional[float] = None
        self._odom_time   = 0.0

        self._ratios: list[float] = []   # chỉ lấy mẫu sau warmup
        self._start_time  = 0.0
        self._running     = False
        self._done        = threading.Event()

        # ── QoS ──────────────────────────────────────────────────────────
        qos_sensor = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # ── Publisher / Subscriber ────────────────────────────────────────
        self._pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_subscription(
            Odometry, '/odom',
            self._cb_odom, qos_sensor)

        # ── Timers ────────────────────────────────────────────────────────
        self._cmd_timer  = self.create_timer(0.05,  self._send_cmd)   # 20 Hz
        self._log_timer  = self.create_timer(0.2,   self._print_live) # 5 Hz
        self._check_timer = self.create_timer(0.1,  self._check_done)

        self._start_time = time.time()
        self._running    = True

        print(f'\n{"="*55}')
        print(f'  Spin Calibration — cmd_wz = {cmd_wz:+.3f} rad/s')
        print(f'  Warmup: {warmup:.0f}s  |  Measurement: {duration:.0f}s')
        print(f'  CMD_SCALE_STEER hiện tại = {CURRENT_CMD_SCALE_STEER}')
        print(f'{"="*55}')
        print(f'  {"t(s)":>5}  {"cmd_wz":>7}  {"odom_wz":>8}  {"ratio":>7}  {"status"}')
        print(f'  {"-"*5}  {"-"*7}  {"-"*8}  {"-"*7}  {"-"*16}')

    # ── Callbacks ─────────────────────────────────────────────────────────────
    def _cb_odom(self, msg: Odometry):
        with self._lock:
            self._odom_wz   = msg.twist.twist.angular.z
            self._odom_time = time.time()

    # ── Command publisher ─────────────────────────────────────────────────────
    def _send_cmd(self):
        if not self._running:
            return
        msg = Twist()
        msg.linear.x  = 0.0
        msg.angular.z = self._cmd_wz
        self._pub_cmd.publish(msg)

    def stop_robot(self):
        msg = Twist()
        self._pub_cmd.publish(msg)
        self._pub_cmd.publish(msg)  # publish 2 lần để chắc chắn

    # ── Live display ─────────────────────────────────────────────────────────
    def _print_live(self):
        if not self._running:
            return

        now     = time.time()
        elapsed = now - self._start_time

        with self._lock:
            odom_wz   = self._odom_wz
            odom_age  = now - self._odom_time

        if odom_wz is None or odom_age > 0.5:
            print(f'  {elapsed:5.1f}s  — đang chờ /odom ...')
            return

        ratio  = odom_wz / self._cmd_wz if abs(self._cmd_wz) > 1e-4 else float('nan')
        in_warmup = elapsed < self._warmup

        if in_warmup:
            status = f'[warmup {self._warmup - elapsed:.0f}s còn lại]'
        else:
            pct = (ratio - 1.0) * 100
            if abs(pct) <= 5:
                status = '✓  trong ngưỡng ±5%'
            elif pct > 0:
                status = f'↑  nhanh hơn {abs(pct):.0f}%'
            else:
                status = f'↓  chậm hơn {abs(pct):.0f}%'
            self._ratios.append(ratio)

        print(f'  {elapsed:5.1f}s  {self._cmd_wz:+7.3f}  {odom_wz:+8.3f}  {ratio:7.4f}  {status}')

    # ── Check done ────────────────────────────────────────────────────────────
    def _check_done(self):
        if not self._running:
            return
        elapsed = time.time() - self._start_time
        if elapsed >= self._warmup + self._duration:
            self._running = False
            self._done.set()

    def wait_until_done(self):
        self._done.wait()

    # ── Summary ───────────────────────────────────────────────────────────────
    def print_summary(self):
        print(f'\n{"="*55}')
        n = len(self._ratios)

        if n < 5:
            print('  Không đủ mẫu để tính. Kiểm tra /odom có publish không.')
            return

        avg    = statistics.mean(self._ratios)
        median = statistics.median(self._ratios)
        stdev  = statistics.stdev(self._ratios)
        pct    = (avg - 1.0) * 100

        print(f'  SUMMARY — {n} mẫu (sau warmup {self._warmup:.0f}s)')
        print(f'  {"avg ratio":>12} = {avg:.4f}   ({pct:+.1f}% so với lý tưởng)')
        print(f'  {"median ratio":>12} = {median:.4f}')
        print(f'  {"stdev":>12} = {stdev:.4f}')
        print(f'{"─"*55}')

        if abs(pct) <= 5:
            print(f'  [OK]  CMD_SCALE_STEER = {CURRENT_CMD_SCALE_STEER} nằm trong ngưỡng ±5%.')
            print(f'        Không cần điều chỉnh.')
        else:
            new_scale = round(CURRENT_CMD_SCALE_STEER / avg)
            print(f'  [FIX] Robot xoay {"nhanh" if pct > 0 else "chậm"} hơn {abs(pct):.1f}% so với lệnh.')
            print(f'        Gợi ý: CMD_SCALE_STEER = {new_scale}  (current = {CURRENT_CMD_SCALE_STEER})')
            print()
            # Hướng dẫn thay đổi
            print(f'  Để áp dụng, sửa wheel_odom_node.py:')
            print(f'    CMD_SCALE_STEER = {new_scale}')

        print(f'{"="*55}\n')


# ─────────────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(
        description='Xoay robot tại chỗ để kiểm tra CMD_SCALE_STEER')
    parser.add_argument('--wz',       type=float, default=0.4,
                        metavar='RAD_S',
                        help='Vận tốc góc (rad/s), mặc định %(default)s')
    parser.add_argument('--duration', type=float, default=10.0,
                        metavar='SEC',
                        help='Thời gian đo (s) sau warmup, mặc định %(default)s')
    parser.add_argument('--warmup',   type=float, default=2.0,
                        metavar='SEC',
                        help='Thời gian bỏ qua lúc khởi động (s), mặc định %(default)s')
    parser.add_argument('--dir',      choices=['ccw', 'cw'], default='ccw',
                        help='Chiều xoay: ccw = ngược KĐH (wz>0), cw = theo KĐH (wz<0)')
    args = parser.parse_args()

    cmd_wz = abs(args.wz) if args.dir == 'ccw' else -abs(args.wz)

    rclpy.init()
    node = SpinCalibrator(
        cmd_wz=cmd_wz,
        duration=args.duration,
        warmup=args.warmup,
    )

    _stopped = threading.Event()

    def _shutdown(reason: str):
        if _stopped.is_set():
            return
        _stopped.set()
        node._running = False
        print(f'\n[{reason}] Dừng robot...')
        node.stop_robot()
        time.sleep(0.2)
        node.print_summary()
        try:
            rclpy.shutdown()
        except Exception:
            pass

    signal.signal(signal.SIGINT, lambda s, f: _shutdown('Ctrl+C'))

    # Spin ROS trong thread riêng
    spin_thread = threading.Thread(
        target=lambda: rclpy.spin(node), daemon=True)
    spin_thread.start()

    # Đợi đến khi đo xong
    node.wait_until_done()
    _shutdown('DONE')


if __name__ == '__main__':
    main()
