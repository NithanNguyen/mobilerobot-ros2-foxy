#!/usr/bin/env python3
import math
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range
from rclpy.qos import qos_profile_sensor_data

SENSOR_POSES = {
    'us_top_left':  (0.29,  0.18, 0.0),
    'us_top_right': (0.29, -0.18, 0.0),
    'us_mid_left':  (0.29,  0.18, 0.0),
    'us_mid_right': (0.29, -0.18, 0.0),
    'us_bot_left':  (0.29,  0.18, 0.0),
    'us_bot_right': (0.29, -0.18, 0.0),
}

BEAM_HALF_ANGLE_RAD = math.radians(15.0)
BEAM_RAYS = 7

class UltrasonicFusionNode(Node):

    COSTMAP_RANGE_MAX = 2.0
    FILTER_WINDOW     = 5
    RELIABLE_MAX    = 2.0
    CONSISTENCY_MIN = 3

    def __init__(self):
        super().__init__('ultrasonic_fusion_node')

        self.sensor_names = list(SENSOR_POSES.keys())
        self.buffers      = {n: deque(maxlen=self.FILTER_WINDOW) for n in self.sensor_names}
        self.latest       = {n: float('inf') for n in self.sensor_names}

        for name in self.sensor_names:
            self.create_subscription(
                Range, f'/ultrasonic/{name}',
                lambda msg, n=name: self._range_cb(msg, n), 10)

        self.scan_pub = self.create_publisher(
            LaserScan, '/ultrasonic_scan', qos_profile_sensor_data)

        self.create_timer(0.1, self._publish_scan)

        self.get_logger().info('=== Ultrasonic Fusion Node started ===')
        self.get_logger().info(
            f'Sensors: {self.sensor_names} → /ultrasonic_scan → Nav2 local_costmap')

    # ── Sensor callback ────────────────────────────────────────────────────────
    def _range_cb(self, msg: Range, name: str):
        if msg.min_range <= msg.range <= msg.max_range:
            self.buffers[name].append(msg.range)
        elif msg.range >= msg.max_range:
            self.buffers[name].append(float('inf'))

        if self.buffers[name]:
            finite_vals = [v for v in self.buffers[name] if not math.isinf(v)]
            self.latest[name] = float(np.median(finite_vals)) if finite_vals else float('inf')

    def _validated_reading(self, name: str) -> float:
        """
        Return the noise-filtered range [m] for costmap use, or inf when unreliable.

        Rules:
          1. Median >= RELIABLE_MAX  → treat as clear → inf
          2. Median <  RELIABLE_MAX  → require CONSISTENCY_MIN readings in the
             sliding window also < RELIABLE_MAX; otherwise single-sample noise → inf
        """
        median = self.latest[name]

        if math.isinf(median) or median >= self.RELIABLE_MAX:
            return float('inf')

        reliable_count = sum(
            1 for v in self.buffers[name]
            if not math.isinf(v) and v < self.RELIABLE_MAX
        )
        if reliable_count >= self.CONSISTENCY_MIN:
            return median

        self.get_logger().debug(
            f'[NOISE] {name}: median={median:.2f} m, '
            f'consistent={reliable_count}/{len(self.buffers[name])} → discarded'
        )
        return float('inf')

    # ── Publish scan → costmap ─────────────────────────────────────────────────
    def _publish_scan(self):
        num_rays        = 360
        ranges          = [float('inf')] * num_rays
        angle_min       = -math.pi
        angle_increment = 2.0 * math.pi / num_rays

        for name, (sx, sy, yaw) in SENSOR_POSES.items():
            d = self._validated_reading(name)
            if math.isinf(d):
                continue 

            # Spread BEAM_RAYS evenly across the beam cone [-BEAM_HALF_ANGLE, +BEAM_HALF_ANGLE]
            for i in range(BEAM_RAYS):
                t     = i / (BEAM_RAYS - 1)              # 0.0 … 1.0
                delta = yaw + BEAM_HALF_ANGLE_RAD * (2.0 * t - 1.0)

                # Obstacle position in robot 2D frame (base_footprint)
                ox = sx + d * math.cos(delta)
                oy = sy + d * math.sin(delta)

                # Convert to polar from robot centre so we can fill the LaserScan array
                scan_angle = math.atan2(oy, ox)
                scan_dist  = math.hypot(ox, oy)

                if scan_dist > self.COSTMAP_RANGE_MAX:
                    continue

                idx = int(round(
                    (scan_angle - angle_min) / angle_increment
                )) % num_rays
                ranges[idx] = min(ranges[idx], scan_dist)

        scan                 = LaserScan()
        scan.header.stamp    = self.get_clock().now().to_msg()
        scan.header.frame_id = 'base_footprint'
        scan.angle_min       = angle_min
        scan.angle_max       =  math.pi
        scan.angle_increment = angle_increment
        scan.time_increment  = 0.0
        scan.scan_time       = 0.1
        scan.range_min       = 0.01
        scan.range_max       = self.COSTMAP_RANGE_MAX
        scan.ranges          = ranges

        self.scan_pub.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
