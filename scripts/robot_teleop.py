#!/usr/bin/env python3
"""
Standalone keyboard teleop for the mobile robot (run over SSH on the Jetson Xavier).

Publishes geometry_msgs/Twist on `cmd_vel`. Press-and-hold to move, release to stop
(a per-axis watchdog forces 0 after `key_timeout` with no input); linear and angular
watchdogs are independent so you can drive and steer together. Clamping and the
m/s -> STM32 UART conversion are done by wheel_odom_node.py, not here.

Key tokens may ALSO arrive from the ESP32 BLE teleop bridge: jetson_sensor_bridge.py
republishes `$K,<char>` frames as std_msgs/String on `/esp32_teleop_keys`, which this
node subscribes to and feeds through the SAME handler as local keystrokes (so the
0.45 s per-axis watchdog applies identically to phone input).

Usage:
  ros2 run mobile_robot robot_teleop.py                       # publish /cmd_vel only
  ros2 run mobile_robot robot_teleop.py --ros-args \\
      -p start_odom:=true                                     # standalone: also drive the STM32
      -p cmd_vel_topic:=cmd_vel -p publish_rate:=20.0 \\
      -p key_timeout:=0.45 \\
      -p vx_default:=0.10 -p vx_step:=0.05 -p vx_max:=0.60 \\
      -p wz_default:=0.40 -p wz_step:=0.10 -p wz_max:=1.20
  python3 robot_teleop.py --ros-args -p start_odom:=true      # run the source file directly

Key bindings:
  W / Up    : forward        S / Down  : backward
  A / Left  : turn left      D / Right : turn right
  + / -     : increase / decrease linear speed (vx)
  ] / [     : increase / decrease angular speed (wz)
  SPACE / X : immediate emergency stop
  Ctrl-C    : quit
"""

import sys
import os
import time
import select
import termios
import tty
import threading
import importlib.util

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist
from std_msgs.msg import String


def load_wheel_odom_class():
    """Load the WheelOdomNode class from the sibling file (same install dir).

    Standalone mode: the teleop file also starts the STM32-communication node
    itself. Because ament installs wheel_odom_node.py renamed to `wheel_odom_node`
    (without the .py), we load it by file path so it works both via `ros2 run` and
    via `python3` directly. Returns None if not found.
    """
    here = os.path.dirname(os.path.abspath(__file__))
    for cand in ('wheel_odom_node.py', 'wheel_odom_node'):
        path = os.path.join(here, cand)
        if os.path.isfile(path):
            spec = importlib.util.spec_from_file_location('wheel_odom_node', path)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)   # name != '__main__' so main() does NOT run
            return getattr(module, 'WheelOdomNode', None)
    return None


# ── Key classification (normalized: arrows -> token 'UP'/'DOWN'/'LEFT'/'RIGHT') ──
FORWARD_KEYS  = {'w', 'W', 'UP'}
BACKWARD_KEYS = {'s', 'S', 'DOWN'}
LEFT_KEYS     = {'a', 'A', 'LEFT'}      # turn left  -> wz positive (REP-103)
RIGHT_KEYS    = {'d', 'D', 'RIGHT'}     # turn right -> wz negative
STOP_KEYS     = {' ', 'x', 'X'}
LIN_UP_KEYS   = {'+', '='}
LIN_DOWN_KEYS = {'-', '_'}
ANG_UP_KEYS   = {']'}
ANG_DOWN_KEYS = {'['}
QUIT_KEYS     = {'\x03'}                 # Ctrl-C

ARROW_MAP = {65: 'UP', 66: 'DOWN', 67: 'RIGHT', 68: 'LEFT'}  # \x1b[A.. -> token


def parse_tokens(data: bytes):
    """Split a raw byte string from stdin into a list of key tokens.

    Handles arrow keys (escape sequences like b'\\x1b[A'). Because auto-repeat can
    pack several bytes into a single read, the function walks the whole buffer.
    """
    tokens = []
    i = 0
    n = len(data)
    while i < n:
        b = data[i]
        if b == 0x1b:  # ESC: try to capture an arrow-key escape sequence
            if i + 2 < n and data[i + 1] == ord('['):
                arrow = ARROW_MAP.get(data[i + 2])
                if arrow:
                    tokens.append(arrow)
                    i += 3
                    continue
            # Lone ESC / unknown sequence -> skip the ESC byte, treat as no command
            i += 1
            continue
        tokens.append(chr(b))
        i += 1
    return tokens


class RobotKeyTeleop(Node):
    def __init__(self):
        super().__init__('robot_key_teleop')

        # ── Parameters (tunable at runtime: --ros-args -p ...) ──
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        # start_odom=True: also start wheel_odom_node (sends UART to the STM32) in
        # the same process -> you only need to run this one file.
        # DEFAULT False for SAFETY: if bringup/nav2 already runs wheel_odom_node,
        # opening it again would fight over /dev/ttyWheel (serial conflict). To
        # drive STANDALONE (without bringup) run with -p start_odom:=true.
        self.declare_parameter('start_odom', False)
        self.declare_parameter('publish_rate', 20.0)   # [Hz] publish frequency
        self.declare_parameter('key_timeout', 0.45)    # [s] per-axis watchdog
        self.declare_parameter('vx_default', 0.20)     # [m/s] slow for safety
        self.declare_parameter('vx_step', 0.05)
        self.declare_parameter('vx_min', 0.05)
        self.declare_parameter('vx_max', 0.60)         # matches MAX_LINEAR_VEL
        self.declare_parameter('wz_default', 0.40)     # [rad/s] slow for safety
        self.declare_parameter('wz_step', 0.10)
        self.declare_parameter('wz_min', 0.10)
        self.declare_parameter('wz_max', 1.20)         # < MAX_ANGULAR_VEL (1.5)

        topic              = self.get_parameter('cmd_vel_topic').value
        self.start_odom    = bool(self.get_parameter('start_odom').value)
        self.publish_rate  = float(self.get_parameter('publish_rate').value)
        self.key_timeout   = float(self.get_parameter('key_timeout').value)
        self.vx_set        = float(self.get_parameter('vx_default').value)
        self.vx_step       = float(self.get_parameter('vx_step').value)
        self.vx_min        = float(self.get_parameter('vx_min').value)
        self.vx_max        = float(self.get_parameter('vx_max').value)
        self.wz_set        = float(self.get_parameter('wz_default').value)
        self.wz_step       = float(self.get_parameter('wz_step').value)
        self.wz_min        = float(self.get_parameter('wz_min').value)
        self.wz_max        = float(self.get_parameter('wz_max').value)

        self.publisher_ = self.create_publisher(Twist, topic, 10)

        # ESP32 BLE teleop input — same tokens as the keyboard, via the bridge.
        self.declare_parameter('esp32_keys_topic', 'esp32_teleop_keys')
        topic_keys = self.get_parameter('esp32_keys_topic').value
        self.create_subscription(String, topic_keys, self._esp32_key_callback, 20)

        # ── Control state ──
        self.lin_dir = 0          # -1 / 0 / +1
        self.ang_dir = 0          # -1 / 0 / +1
        self.lin_deadline = 0.0   # monotonic time when forward/backward expires
        self.ang_deadline = 0.0   # monotonic time when turn expires
        self._last_status = None  # so we only print when the state changes

        self.settings = termios.tcgetattr(sys.stdin)

    # ── Read & handle keys ──────────────────────────────────────────────
    def _read_tokens(self, timeout):
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if not rlist:
            return []
        data = os.read(sys.stdin.fileno(), 64)
        return parse_tokens(data)

    def _esp32_key_callback(self, msg: String) -> None:
        """Receive a single key token from ESP32 (via jetson_sensor_bridge)."""
        if not msg.data:
            return
        # Re-use the existing keyboard handler — same semantics + same watchdog.
        self._handle_token(msg.data[0])

    def _handle_token(self, tok):
        """Apply a single key token. Returns False if a quit was requested."""
        now = time.monotonic()

        if tok in QUIT_KEYS:
            return False

        if tok in STOP_KEYS:
            # Emergency stop: force both axes to 0 immediately
            self.lin_dir = 0
            self.ang_dir = 0
            self.lin_deadline = 0.0
            self.ang_deadline = 0.0
            return True

        if tok in FORWARD_KEYS:
            self.lin_dir = 1
            self.lin_deadline = now + self.key_timeout
        elif tok in BACKWARD_KEYS:
            self.lin_dir = -1
            self.lin_deadline = now + self.key_timeout
        elif tok in LEFT_KEYS:
            self.ang_dir = 1
            self.ang_deadline = now + self.key_timeout
        elif tok in RIGHT_KEYS:
            self.ang_dir = -1
            self.ang_deadline = now + self.key_timeout
        elif tok in LIN_UP_KEYS:
            self.vx_set = min(self.vx_max, round(self.vx_set + self.vx_step, 3))
        elif tok in LIN_DOWN_KEYS:
            self.vx_set = max(self.vx_min, round(self.vx_set - self.vx_step, 3))
        elif tok in ANG_UP_KEYS:
            self.wz_set = min(self.wz_max, round(self.wz_set + self.wz_step, 3))
        elif tok in ANG_DOWN_KEYS:
            self.wz_set = max(self.wz_min, round(self.wz_set - self.wz_step, 3))
        # any other token -> ignore
        return True

    # ── Main loop ───────────────────────────────────────────────────────
    def run_loop(self):
        self._print_help()
        period = 1.0 / self.publish_rate

        while rclpy.ok():
            # Read keys, blocking at most one period to keep the publish rate
            for tok in self._read_tokens(period):
                if not self._handle_token(tok):
                    return  # quit

            now = time.monotonic()
            # Independent per-axis watchdog: expired -> force to 0 (safety)
            if now > self.lin_deadline:
                self.lin_dir = 0
            if now > self.ang_deadline:
                self.ang_dir = 0

            twist = Twist()
            twist.linear.x = self.lin_dir * self.vx_set
            twist.angular.z = self.ang_dir * self.wz_set
            self.publisher_.publish(twist)

            self._print_status(twist)

    # ── Display (raw mode needs '\r' to avoid staircasing) ───────────────
    def _write_line(self, text):
        sys.stdout.write('\r' + text + '\x1b[K')
        sys.stdout.flush()

    def _print_help(self):
        keys = 'Key bindings:' + __doc__.split('Key bindings:')[1]
        sys.stdout.write('\r\n')
        sys.stdout.write(keys.replace('\n', '\r\n'))
        sys.stdout.write('\r\n')
        sys.stdout.flush()

    def _print_status(self, twist):
        status = (self.lin_dir, self.ang_dir,
                  round(self.vx_set, 3), round(self.wz_set, 3))
        if status == self._last_status:
            return
        self._last_status = status

        if self.lin_dir == 0 and self.ang_dir == 0:
            move = 'STOP'
        else:
            parts = []
            if self.lin_dir > 0:
                parts.append('FWD')
            elif self.lin_dir < 0:
                parts.append('BACK')
            if self.ang_dir > 0:
                parts.append('LEFT')
            elif self.ang_dir < 0:
                parts.append('RIGHT')
            move = '+'.join(parts)

        self._write_line(
            f'[{move:>10}]  vx={twist.linear.x:+.2f} m/s  '
            f'wz={twist.angular.z:+.2f} rad/s  '
            f'(set vx={self.vx_set:.2f}, wz={self.wz_set:.2f})'
        )

    # ── Cleanup ───────────────────────────────────────────────────────────
    def stop_robot(self):
        """Publish Twist = 0 a few times to make sure the STM32 gets the stop."""
        try:
            for _ in range(5):
                self.publisher_.publish(Twist())
                time.sleep(0.02)
        except Exception:
            pass


def _start_wheel_odom(teleop):
    """Start WheelOdomNode on a background thread. Returns (executor, node, thread).

    If the class can't be found or the serial port fails to open (STM32 not
    plugged in / not powered), log a warning and return (None, None, None) — the
    teleop still runs in publish-only mode so you can test without hardware.
    """
    OdomCls = load_wheel_odom_class()
    if OdomCls is None:
        teleop.get_logger().warn(
            'wheel_odom_node not found — running teleop standalone (publishes /cmd_vel only).')
        return None, None, None

    try:
        odom_node = OdomCls()
    except Exception as e:   # serial.SerialException... -> still let teleop run
        teleop.get_logger().warn(
            f'Failed to start wheel_odom_node ({e}). '
            f'Running teleop standalone — NOT sending UART to the STM32.')
        return None, None, None

    executor = SingleThreadedExecutor()
    executor.add_node(odom_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    teleop.get_logger().info('wheel_odom_node started — commands will reach the STM32.')
    return executor, odom_node, thread


def main(args=None):
    rclpy.init(args=args)
    node = RobotKeyTeleop()

    # Background spin so the /esp32_teleop_keys subscription actually fires —
    # run_loop() below publishes from a manual loop and never calls spin itself.
    spin_executor = SingleThreadedExecutor()
    spin_executor.add_node(node)
    spin_thread = threading.Thread(target=spin_executor.spin, daemon=True)
    spin_thread.start()

    executor = odom_node = odom_thread = None
    if node.start_odom:
        executor, odom_node, odom_thread = _start_wheel_odom(node)

    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setraw(sys.stdin.fileno())
        node.run_loop()
    except KeyboardInterrupt:
        pass
    finally:
        # Restore the terminal BEFORE printing/stopping so output isn't garbled
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        # stop_robot() runs WHILE odom is still spinning -> the STOP reaches the STM32
        node.stop_robot()
        node.get_logger().info('Teleop stopped — STOP command sent.')

        # Stop the teleop spin thread before destroying the node.
        spin_executor.shutdown()
        spin_thread.join(timeout=2.0)

        if executor is not None:
            executor.shutdown()          # stop the background odom spin thread
            odom_thread.join(timeout=2.0)
            odom_node.destroy_node()

        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
