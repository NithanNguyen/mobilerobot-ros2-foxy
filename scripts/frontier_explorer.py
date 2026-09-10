#!/usr/bin/env python3
"""
Frontier-based autonomous exploration node.

Key fixes:
- Use the robot's real pose from TF instead of (0, 0).
- Do not mark a frontier as visited until the goal is actually reached.
- Avoid spinning 360° at every waypoint; use a shorter, slower scan spin.
- Add goal back-off so the robot stops slightly before the frontier boundary.
- Keep a small retry/skip history so the node does not get stuck on one frontier.
"""

from __future__ import annotations

import math
import time
from enum import Enum
from collections import deque
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose, Spin
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


class ExploreState(Enum):
    IDLE = 0
    EXPLORE = 1
    MOVING = 2
    SPINNING = 3
    DONE = 4


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')

        # ---------------- Parameters ----------------
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')

        # Topic for frontier detection. MUST be the raw SLAM map (/map), which
        # contains -1 for unknown cells. The Nav2 global costmap defaults to
        # track_unknown_space:false, so it never has -1 and frontier detection
        # would always return empty.
        self.declare_parameter('map_topic', '/map')

        self.declare_parameter('min_frontier_size', 8)      # cells
        self.declare_parameter('goal_tolerance', 0.35)      # m
        self.declare_parameter('exploration_timeout', 900.0)  # s
        self.declare_parameter('explore_timer_period', 1.5)  # s

        # Frontier scoring / safety
        self.declare_parameter('frontier_approach_offset', 0.35)  # m back from centroid toward robot
        self.declare_parameter('frontier_retry_limit', 2)
        self.declare_parameter('frontier_size_weight', 0.01)      # larger frontier = slightly preferred
        self.declare_parameter('frontier_min_distance', 0.60)      # ignore frontiers too close to robot

        # Post-arrival scan
        self.declare_parameter('spin_on_arrival', False)
        self.declare_parameter('spin_yaw', 3.14)                    # 180 deg; faster than full 360°
        self.declare_parameter('spin_timeout', 20.0)                # s

        # FOV configuration — matches physical LiDAR mounting on this robot
        self.declare_parameter('fov_front_half', math.pi / 3)     # 60° = half of 120° front arc
        self.declare_parameter('fov_side_max',   math.pi * 2 / 3) # 120° — beyond this is edge zone
        self.declare_parameter('fov_blind_threshold', math.pi * 5 / 6) # 150° — beyond this = blind zone, filter out

        # Hybrid scoring weights
        self.declare_parameter('w_dist',      3.0)   # distance to frontier (lower = closer preferred)
        self.declare_parameter('w_rot',       2.5)   # rotation cost (penalizes large heading changes)
        self.declare_parameter('w_fov',       2.0)   # FOV zone penalty multiplier
        self.declare_parameter('w_clearance', 1.5)   # clearance reward (rewards open corridor centers)

        # FOV zone penalty values (added to score, so higher = less preferred)
        self.declare_parameter('fov_penalty_side', 1.5)  # for 60°–120° zone
        self.declare_parameter('fov_penalty_edge', 3.0)  # for 120°–150° zone

        # Clearance-aware goal placement
        self.declare_parameter('clearance_search_radius', 0.80)  # meters to search around frontier centroid

        # Rear blind zone scan trigger (replaces always-on spin_on_arrival)
        self.declare_parameter('rear_scan_trigger_distance', 2.5)  # meters traveled before triggering rear scan
        self.declare_parameter('rear_scan_yaw', 1.8)               # radians — covers most of blind zone

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.map_topic = self.get_parameter('map_topic').value
        self.min_frontier_size = int(self.get_parameter('min_frontier_size').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self.exploration_timeout = float(self.get_parameter('exploration_timeout').value)
        self.explore_timer_period = float(self.get_parameter('explore_timer_period').value)
        self.frontier_approach_offset = float(self.get_parameter('frontier_approach_offset').value)
        self.frontier_retry_limit = int(self.get_parameter('frontier_retry_limit').value)
        self.frontier_size_weight = float(self.get_parameter('frontier_size_weight').value)
        self.frontier_min_distance = float(self.get_parameter('frontier_min_distance').value)
        self.spin_on_arrival = bool(self.get_parameter('spin_on_arrival').value)
        self.spin_yaw = float(self.get_parameter('spin_yaw').value)
        self.spin_timeout = float(self.get_parameter('spin_timeout').value)

        self.fov_front_half = float(self.get_parameter('fov_front_half').value)
        self.fov_side_max = float(self.get_parameter('fov_side_max').value)
        self.fov_blind_threshold = float(self.get_parameter('fov_blind_threshold').value)
        self.w_dist = float(self.get_parameter('w_dist').value)
        self.w_rot = float(self.get_parameter('w_rot').value)
        self.w_fov = float(self.get_parameter('w_fov').value)
        self.w_clearance = float(self.get_parameter('w_clearance').value)
        self.fov_penalty_side = float(self.get_parameter('fov_penalty_side').value)
        self.fov_penalty_edge = float(self.get_parameter('fov_penalty_edge').value)
        self.clearance_search_radius = float(self.get_parameter('clearance_search_radius').value)
        self.rear_scan_trigger_distance = float(self.get_parameter('rear_scan_trigger_distance').value)
        self.rear_scan_yaw = float(self.get_parameter('rear_scan_yaw').value)

        # ---------------- State ----------------
        self.state = ExploreState.IDLE
        self.map_data: OccupancyGrid | None = None
        self.exploration_start_time = None

        # visited/frontier bookkeeping
        self.visited_frontiers = set()
        self.failed_frontiers = {}
        self.active_goal_key = None

        # rear blind-zone scan trigger bookkeeping
        self._travel_accumulator: float = 0.0   # meters since last rear scan
        self._last_travel_pose: Optional[Tuple[float, float]] = None

        # nav / spin handles
        self._nav_goal_handle = None
        self._spin_goal_handle = None

        # ---------------- TF ----------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---------------- QoS for latched map ----------------
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid, self.map_topic, self.map_callback, map_qos
        )
        self.frontier_pub = self.create_publisher(MarkerArray, '/frontiers', 10)

        # ---------------- Services ----------------
        self.start_srv = self.create_service(Trigger, '/explore/start', self.start_callback)
        self.stop_srv = self.create_service(Trigger, '/explore/stop', self.stop_callback)

        # ---------------- Action clients ----------------
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.spin_client = ActionClient(self, Spin, 'spin')

        # ---------------- Timer ----------------
        self.explore_timer = self.create_timer(self.explore_timer_period, self.explore_loop)

        self.get_logger().info('FrontierExplorer ready. Call /explore/start to begin.')

    # =========================================================
    # Utils
    # =========================================================
    @staticmethod
    def yaw_to_quaternion(yaw: float) -> Quaternion:
        q = Quaternion()
        half = yaw * 0.5
        q.z = math.sin(half)
        q.w = math.cos(half)
        return q

    def get_robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time()
            )
        except TransformException as ex:
            self.get_logger().warn(f'Cannot get TF {self.map_frame}->{self.base_frame}: {ex}')
            return None

        x = tf.transform.translation.x
        y = tf.transform.translation.y

        # quaternion -> yaw
        q = tf.transform.rotation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return x, y, yaw

    def reset_session(self):
        self.visited_frontiers.clear()
        self.failed_frontiers.clear()
        self.active_goal_key = None
        self._nav_goal_handle = None
        self._spin_goal_handle = None
        self._travel_accumulator = 0.0
        self._last_travel_pose = None
        self.exploration_start_time = time.monotonic()

    def cancel_all_actions(self):
        try:
            self.nav_client.cancel_all_goals_async()
        except Exception:
            pass
        try:
            self.spin_client.cancel_all_goals_async()
        except Exception:
            pass

    # =========================================================
    # ROS callbacks
    # =========================================================
    def map_callback(self, msg: OccupancyGrid):
        self.map_data = msg

    def start_callback(self, request, response):
        if self.state not in (ExploreState.IDLE, ExploreState.DONE):
            response.success = False
            response.message = f'Already running in state: {self.state.name}'
            return response

        self.reset_session()
        self.state = ExploreState.EXPLORE
        response.success = True
        response.message = 'Exploration started!'
        self.get_logger().info('>>> Exploration STARTED')
        return response

    def stop_callback(self, request, response):
        self.cancel_all_actions()
        self.state = ExploreState.IDLE
        self.active_goal_key = None
        self.get_logger().info('>>> Exploration STOPPED by user.')
        response.success = True
        response.message = 'Exploration stopped.'
        return response

    # =========================================================
    # Main loop
    # =========================================================
    def explore_loop(self):
        if self.state not in (ExploreState.EXPLORE,):
            return

        if self.map_data is None:
            if not hasattr(self, '_last_wait_warn') or (time.monotonic() - self._last_wait_warn) > 5.0:
                self._last_wait_warn = time.monotonic()
                self.get_logger().warn('Waiting for map...')
            return

        if self.exploration_start_time is not None:
            elapsed = time.monotonic() - self.exploration_start_time
            if elapsed > self.exploration_timeout:
                self.get_logger().warn('Exploration timeout reached. Stopping.')
                self.state = ExploreState.DONE
                return

        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            return

        frontiers = self.detect_frontiers(self.map_data)
        if not frontiers:
            self.get_logger().info('No frontiers found. Exploration COMPLETE.')
            self.state = ExploreState.DONE
            return

        self.publish_frontier_markers(frontiers)

        goal = self.select_best_frontier(frontiers, robot_pose)
        if goal is None:
            self.get_logger().info('All remaining frontiers were already tried. Exploration COMPLETE.')
            self.state = ExploreState.DONE
            return

        gx, gy, key = goal
        self.active_goal_key = key

        self.get_logger().info(f'Navigating to frontier: ({gx:.2f}, {gy:.2f})')
        self.state = ExploreState.MOVING
        self.send_nav_goal(gx, gy)

    # =========================================================
    # Frontier detection
    # =========================================================
    def detect_frontiers(self, map_msg: OccupancyGrid):
        """
        Frontier = free cell (0) with at least one 4-neighbor unknown (255 or -1).
        Returns [(wx, wy, size), ...]
        """
        info = map_msg.info
        width = info.width
        height = info.height
        res = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y
        data = np.array(map_msg.data, dtype=np.int16).reshape((height, width))

        frontier_cells = []
        for r in range(1, height - 1):
            for c in range(1, width - 1):
                val = data[r, c]
                if val < 0 or val > 20:
                    continue
                
                neighbors = (data[r - 1, c], data[r + 1, c], data[r, c - 1], data[r, c + 1])
                if -1 in neighbors:
                    frontier_cells.append((r, c))

        if not frontier_cells:
            self.get_logger().warn(f'No frontier cells found! Unique values in costmap: {np.unique(data)}')
            return []
        cell_set = set(frontier_cells)
        visited = set()
        clusters = []

        for seed in frontier_cells:
            if seed in visited:
                continue

            cluster = []
            q = deque([seed])

            while q:
                cur = q.popleft()
                if cur in visited:
                    continue
                visited.add(cur)

                if cur not in cell_set:
                    continue

                cluster.append(cur)
                r, c = cur
                for nr, nc in ((r - 1, c), (r + 1, c), (r, c - 1), (r, c + 1)):
                    if 0 <= nr < height and 0 <= nc < width and (nr, nc) not in visited:
                        q.append((nr, nc))

            if len(cluster) < self.min_frontier_size:
                continue

            cr = float(np.mean([p[0] for p in cluster]))
            cc = float(np.mean([p[1] for p in cluster]))
            wx = cc * res + ox + res * 0.5
            wy = cr * res + oy + res * 0.5
            clusters.append((wx, wy, len(cluster)))

        return clusters

    # =========================================================
    # Frontier selection
    # =========================================================
    def select_best_frontier(self, frontiers, robot_pose):
        """
        Choose the best frontier using a hybrid score that blends distance,
        rotation cost, LiDAR FOV zone penalty and centroid clearance, while
        hard-filtering frontiers that fall inside the rear blind zone.
        Return (goal_x, goal_y, key) or None.
        """
        robot_x, robot_y, robot_yaw = robot_pose
        candidates = []

        info = self.map_data.info
        res = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y
        width = info.width
        height = info.height
        grid = np.array(self.map_data.data, dtype=np.int16).reshape((height, width))

        for wx, wy, size in frontiers:
            dist = math.hypot(wx - robot_x, wy - robot_y)
            key = (round(wx, 2), round(wy, 2))

            # 1) Skip already-handled or over-retried frontiers
            if key in self.visited_frontiers:
                continue
            if self.failed_frontiers.get(key, 0) >= self.frontier_retry_limit:
                continue
            # 2) Skip frontiers too close to the robot
            if dist < self.frontier_min_distance:
                continue

            # 3) Bearing to frontier relative to robot heading, normalized to [-pi, pi]
            bearing = math.atan2(wy - robot_y, wx - robot_x)
            delta_yaw = bearing - robot_yaw
            delta_yaw = math.atan2(math.sin(delta_yaw), math.cos(delta_yaw))

            # 4) HARD FILTER — rear blind zone (robot cannot see this direction)
            if abs(delta_yaw) > self.fov_blind_threshold:
                continue

            # 5) FOV zone penalty
            if abs(delta_yaw) <= self.fov_front_half:
                fov_penalty = 0.0
            elif abs(delta_yaw) <= self.fov_side_max:
                fov_penalty = self.fov_penalty_side
            else:
                fov_penalty = self.fov_penalty_edge

            # 6) Centroid clearance proxy from costmap value at the frontier centroid
            col = int((wx - ox) / res)
            row = int((wy - oy) / res)
            if 0 <= row < height and 0 <= col < width:
                val = int(grid[row, col])
                if 0 <= val <= 100:
                    centroid_clearance = (100.0 - float(val)) / 100.0
                else:
                    centroid_clearance = 0.0
            else:
                centroid_clearance = 0.0

            # 7) Composite score (LOWER = better)
            score = (self.w_dist * dist
                     + self.w_rot * abs(delta_yaw)
                     + self.w_fov * fov_penalty
                     - self.w_clearance * centroid_clearance)

            # 8) Add to candidates
            candidates.append((score, wx, wy, key))

        if not candidates:
            return None

        candidates.sort(key=lambda x: x[0])
        _, wx, wy, key = candidates[0]

        goal_x, goal_y = self.clearance_aware_goal((wx, wy), robot_pose)
        return goal_x, goal_y, key

    def clearance_aware_goal(
        self,
        frontier_centroid: Tuple[float, float],
        robot_pose: Tuple[float, float, float]
    ) -> Tuple[float, float]:
        """
        Primary goal-placement strategy: search a small window around the frontier
        centroid for the truly-free cell with the most clearance (open corridor
        center) and aim there. Falls back to approach_frontier() backoff if no free
        cell is found in the window.
        """
        fx, fy = frontier_centroid
        res = self.map_data.info.resolution
        ox = self.map_data.info.origin.position.x
        oy = self.map_data.info.origin.position.y
        width = self.map_data.info.width
        height = self.map_data.info.height
        data = np.array(self.map_data.data, dtype=np.int16).reshape((height, width))

        search_cells = int(self.clearance_search_radius / res)
        fc = int((fx - ox) / res)
        fr = int((fy - oy) / res)

        best_clearance = -1
        best_wx, best_wy = fx, fy
        found_free = False

        for dr in range(-search_cells, search_cells + 1):
            for dc in range(-search_cells, search_cells + 1):
                r, c = fr + dr, fc + dc
                if not (0 <= r < height and 0 <= c < width):
                    continue
                val = int(data[r, c])
                # Only consider cells that are truly free (not inflated, not unknown)
                if val < 0 or val > 10:
                    continue
                clearance = 255 - val  # higher val = closer to obstacle = less clearance
                if clearance > best_clearance:
                    best_clearance = clearance
                    best_wx = c * res + ox + res * 0.5
                    best_wy = r * res + oy + res * 0.5
                    found_free = True

        if found_free:
            return best_wx, best_wy
        else:
            # Fallback: use original backoff logic
            return self.approach_frontier(robot_pose, (fx, fy))

    def approach_frontier(self, robot_pose, frontier_xy):
        """
        Move to a point slightly before the frontier centroid so the robot stays in
        known free space and does not try to enter unknown cells.
        """
        rx, ry, _ = robot_pose
        fx, fy = frontier_xy

        dx = fx - rx
        dy = fy - ry
        dist = math.hypot(dx, dy)
        backoff_dist = 0.6

        if dist > backoff_dist:
            gx = fx - (dx / dist) * backoff_dist
            gy = fy - (dy / dist) * backoff_dist
        else:
            gx = rx
            gy = ry
        return gx, gy

    # =========================================================
    # Actions
    # =========================================================
    def send_nav_goal(self, x: float, y: float):
        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            self.state = ExploreState.EXPLORE
            return

        rx, ry, _ = robot_pose
        yaw = math.atan2(y - ry, x - rx)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = self.map_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.orientation = self.yaw_to_quaternion(yaw)

        if not self.nav_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().warn('navigate_to_pose action server not available.')
            self.state = ExploreState.EXPLORE
            return

        future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.nav_feedback_callback
        )
        future.add_done_callback(self.nav_goal_response_callback)

    def nav_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal REJECTED by Nav2.')
            if self.active_goal_key is not None:
                self.failed_frontiers[self.active_goal_key] = self.failed_frontiers.get(self.active_goal_key, 0) + 1
            self.active_goal_key = None
            self.state = ExploreState.EXPLORE
            return

        self._nav_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_result_callback)

    def nav_result_callback(self, future):
        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal reached.')

            # Accumulate travel distance since last rear scan
            current_pose = self.get_robot_pose()
            if current_pose is not None:
                cx, cy, _ = current_pose
                if self._last_travel_pose is not None:
                    lx, ly = self._last_travel_pose
                    self._travel_accumulator += math.hypot(cx - lx, cy - ly)
                self._last_travel_pose = (cx, cy)

            # Trigger rear scan only when blind zone has not been covered for a while
            if (self._travel_accumulator >= self.rear_scan_trigger_distance
                    and self.spin_on_arrival):
                self._travel_accumulator = 0.0
                self.state = ExploreState.SPINNING
                # Override spin_yaw with rear_scan_yaw for this targeted scan
                original_yaw = self.spin_yaw
                self.spin_yaw = self.rear_scan_yaw
                self.send_spin_goal()
                self.spin_yaw = original_yaw
                return

            # Otherwise go straight to next frontier
            if self.active_goal_key is not None:
                self.visited_frontiers.add(self.active_goal_key)
                self.active_goal_key = None
            self.state = ExploreState.EXPLORE
            return

        self.get_logger().warn(f'Goal failed (status={status}).')
        if self.active_goal_key is not None:
            self.failed_frontiers[self.active_goal_key] = self.failed_frontiers.get(self.active_goal_key, 0) + 1
            if self.failed_frontiers[self.active_goal_key] >= self.frontier_retry_limit:
                self.visited_frontiers.add(self.active_goal_key)
        self.active_goal_key = None
        self.state = ExploreState.EXPLORE

    def send_spin_goal(self):
        spin_goal = Spin.Goal()
        spin_goal.target_yaw = float(self.spin_yaw)

        if not self.spin_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().warn('spin action server not available. Continuing.')
            if self.active_goal_key is not None:
                self.visited_frontiers.add(self.active_goal_key)
                self.active_goal_key = None
            self.state = ExploreState.EXPLORE
            return

        future = self.spin_client.send_goal_async(spin_goal)
        future.add_done_callback(self.spin_response_callback)

        # Spin timeout watchdog
        self._spin_started_at = time.monotonic()

    def spin_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Spin goal rejected.')
            if self.active_goal_key is not None:
                self.visited_frontiers.add(self.active_goal_key)
                self.active_goal_key = None
            self.state = ExploreState.EXPLORE
            return

        self._spin_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.spin_result_callback)

    def spin_result_callback(self, future):
        _ = future.result()
        self.get_logger().info('Spin complete. Searching next frontier...')

        if self.active_goal_key is not None:
            self.visited_frontiers.add(self.active_goal_key)
            self.active_goal_key = None

        self.state = ExploreState.EXPLORE

    def nav_feedback_callback(self, feedback_msg):
        # Intentionally light to keep Jetson Xavier CPU load low.
        return

    # =========================================================
    # Visualization
    # =========================================================
    def publish_frontier_markers(self, frontiers):
        marker_array = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        for i, (wx, wy, size) in enumerate(frontiers):
            m = Marker()
            m.header.frame_id = self.map_frame
            m.header.stamp = stamp
            m.ns = 'frontiers'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position = Point(x=float(wx), y=float(wy), z=0.10)
            m.pose.orientation.w = 1.0

            s = min(0.45, max(0.12, size * 0.01))
            m.scale.x = s
            m.scale.y = s
            m.scale.z = s

            m.color.r = 1.0
            m.color.g = 0.55
            m.color.b = 0.0
            m.color.a = 0.85
            m.lifetime = Duration(seconds=2).to_msg()
            marker_array.markers.append(m)

        self.frontier_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()