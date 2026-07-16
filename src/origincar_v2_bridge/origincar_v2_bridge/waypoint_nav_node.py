"""
Waypoint Navigation Node (Inertial Navigation)
================================================
Uses EKF-fused odometry (/odom_combined) to navigate through a list of
predefined waypoints. At each waypoint, can trigger a task action.

Subscriptions:
  /odom_combined    (nav_msgs/Odometry) - fused position
  /nav_command      (std_msgs/String)   - "start", "pause", "resume", "reset"

Publications:
  /cmd_vel          (geometry_msgs/Twist) - velocity commands
  /nav_status       (std_msgs/String)     - current navigation state info
  /current_waypoint (std_msgs/Int32)      - index of current target waypoint

Control scheme:
  1. Rotate in place to face next waypoint (heading PID)
  2. Drive forward while correcting heading (dual PID: linear + angular)
  3. When within reach_tolerance of waypoint, mark arrived
  4. Execute task (pause for task_duration seconds)
  5. Move to next waypoint
"""

import math
import threading
from enum import Enum
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int32, Bool

# Inline quaternion → yaw (avoid tf_transformations dependency)
def _quat_to_yaw(x, y, z, w):
    """Extract yaw from quaternion (2D only)."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class NavState(Enum):
    IDLE = 0           # Waiting for start command
    DRIVING = 1        # Driving toward waypoint (steer + throttle)
    ARRIVED = 2        # At waypoint, executing task
    FINISHED = 3       # All waypoints completed
    PAUSED = 4         # Paused by command
    HANDOFF = 5        # Control handed off to vision line-following (final leg)
    VISION_LEAD = 6    # Vision drives from start until QR code is scanned
    QR_TURNAROUND = 7  # Post-QR reverse+right-turn maneuver before resuming odometry nav
    QR_RECENTER = 8
    VLM_CAPTURE = 9


class WaypointNavNode(Node):
    def __init__(self):
        super().__init__('waypoint_nav_node')

        # --- Parameters ---
        self.declare_parameter('waypoints', [0.0, 0.0, 0.0])  # flat list: x1,y1,yaw1, x2,y2,yaw2, ...
        self.declare_parameter('reach_tolerance', 0.05)         # m, distance to consider "arrived"
        self.declare_parameter('angle_tolerance', 0.15)         # rad (~9°), heading OK to go full speed
        self.declare_parameter('max_linear_speed', 0.5)         # m/s
        self.declare_parameter('min_linear_speed', 0.1)         # m/s
        self.declare_parameter('max_angular_speed', 1.5)        # rad/s
        self.declare_parameter('linear_kp', 0.8)
        self.declare_parameter('linear_ki', 0.0)
        self.declare_parameter('linear_kd', 0.1)
        self.declare_parameter('angular_kp', 2.0)
        self.declare_parameter('angular_ki', 0.0)
        self.declare_parameter('angular_kd', 0.3)
        self.declare_parameter('task_duration', 2.0)            # seconds to pause at each waypoint
        self.declare_parameter('slowdown_radius', 0.3)          # m, start decelerating within this radius
        self.declare_parameter('loop', False)                   # loop through waypoints
        self.declare_parameter('auto_start', False)             # start immediately
        # Vision handoff: at this waypoint index, stop odometry nav and hand
        # control to the vision line-following stack (IMU drifts on the final
        # return-to-origin leg after the X-crossing). -1 disables handoff.
        self.declare_parameter('vision_handoff_wp_idx', -1)
        # Lead phase: drive with vision from start until the QR code is scanned
        # at the teardrop, then switch to odometry waypoint following.
        self.declare_parameter('lead_with_vision', False)
        # Waypoint index to resume odometry nav from after the QR is scanned
        # (the teardrop tip). Kept for backward compat; nearest-WP search is used instead.
        self.declare_parameter('odom_resume_wp_idx', 118)
        # Post-QR reverse+right-turn parameters
        self.declare_parameter('qr_turnaround_reverse_speed', 0.2)   # m/s magnitude
        self.declare_parameter('qr_turnaround_angular', 1.5)         # rad/s magnitude (right = negative)
        self.declare_parameter('qr_turnaround_yaw_deg', 80.0)        # total rotation to complete (degrees)
        self.declare_parameter('qr_turnaround_presteer_sec', 0.4)
        self.declare_parameter('qr_recenter_sec', 0.25)
        self.declare_parameter('qr_reverse_lockout_sec', 1.5)
        self.declare_parameter('qr_resume_target_distance', 0.45)
        self.declare_parameter('qr_resume_speed_cap_sec', 0.8)
        self.declare_parameter('qr_resume_speed_cap', 0.35)
        self.declare_parameter('route_branch_wp_idx', 175)
        self.declare_parameter('route_merge_wp_idx', 425)
        self.declare_parameter('clockwise_vlm_wp_idx', 329)
        self.declare_parameter('counterclockwise_vlm_wp_idx', 325)
        self.declare_parameter('vlm_stop_sec', 2.0)
        self.declare_parameter('vlm_capture_delay_sec', 1.0)

        # Load parameters
        self.reach_tol = self.get_parameter('reach_tolerance').value
        self.angle_tol = self.get_parameter('angle_tolerance').value
        self.max_v = self.get_parameter('max_linear_speed').value
        self.min_v = self.get_parameter('min_linear_speed').value
        self.max_w = self.get_parameter('max_angular_speed').value
        self.lin_kp = self.get_parameter('linear_kp').value
        self.lin_ki = self.get_parameter('linear_ki').value
        self.lin_kd = self.get_parameter('linear_kd').value
        self.ang_kp = self.get_parameter('angular_kp').value
        self.ang_ki = self.get_parameter('angular_ki').value
        self.ang_kd = self.get_parameter('angular_kd').value
        self.task_duration = self.get_parameter('task_duration').value
        self.slowdown_radius = self.get_parameter('slowdown_radius').value
        self.loop = self.get_parameter('loop').value
        self.auto_start = self.get_parameter('auto_start').value
        self.vision_handoff_idx = self.get_parameter('vision_handoff_wp_idx').value
        self.lead_with_vision = self.get_parameter('lead_with_vision').value
        self.odom_resume_idx = self.get_parameter('odom_resume_wp_idx').value
        self.qr_reverse_speed = self.get_parameter('qr_turnaround_reverse_speed').value
        self.qr_angular = self.get_parameter('qr_turnaround_angular').value
        self.qr_yaw_threshold = math.radians(self.get_parameter('qr_turnaround_yaw_deg').value)
        self.qr_presteer_sec = self.get_parameter('qr_turnaround_presteer_sec').value
        self.qr_recenter_sec = self.get_parameter('qr_recenter_sec').value
        self.qr_reverse_lockout_sec = self.get_parameter('qr_reverse_lockout_sec').value
        self.qr_resume_target_distance = self.get_parameter('qr_resume_target_distance').value
        self.qr_resume_speed_cap_sec = self.get_parameter('qr_resume_speed_cap_sec').value
        self.qr_resume_speed_cap = self.get_parameter('qr_resume_speed_cap').value
        self.route_branch_idx = int(self.get_parameter('route_branch_wp_idx').value)
        self.route_merge_idx = int(self.get_parameter('route_merge_wp_idx').value)
        self.clockwise_vlm_wp_idx = int(self.get_parameter('clockwise_vlm_wp_idx').value)
        self.counterclockwise_vlm_wp_idx = int(self.get_parameter('counterclockwise_vlm_wp_idx').value)
        self.vlm_stop_sec = max(0.0, float(self.get_parameter('vlm_stop_sec').value))
        self.vlm_capture_delay_sec = max(
            0.0, min(float(self.get_parameter('vlm_capture_delay_sec').value), self.vlm_stop_sec))

        # Parse waypoints from flat list
        wp_flat = self.get_parameter('waypoints').value
        self.waypoints = self._parse_waypoints(wp_flat)
        self.base_waypoints = list(self.waypoints)
        self.route_direction = 'clockwise'

        # Ackermann constraints
        self.declare_parameter('turn_speed_ratio', 0.4)          # When heading error large, v = max_v * ratio
        self.declare_parameter('reverse_threshold', 2.5)         # rad (~143°), if target behind, reverse
        self.declare_parameter('lookahead_distance', 0.30)       # m, lookahead for dense path following
        self.declare_parameter('lookahead_max_distance', 0.65)   # m, higher-speed lookahead cap
        self.declare_parameter('lookahead_speed_gain', 0.45)     # m per m/s, extends lookahead at speed
        self.declare_parameter('relocate_distance_threshold', 0.85)
        self.declare_parameter('relocate_heading_threshold_deg', 120.0)
        self.declare_parameter('relocate_search_window', 55)
        self.declare_parameter('relocate_confirm_cycles', 3)
        self.declare_parameter('relocate_max_jump', 40)
        self.declare_parameter('reverse_soft_threshold_deg', 120.0)
        self.declare_parameter('reverse_soft_distance', 0.85)
        self.declare_parameter('reverse_hard_distance', 0.42)
        self.declare_parameter('reverse_exit_threshold_deg', 75.0)
        self.declare_parameter('curve_slowdown_lookahead', 12)
        self.declare_parameter('curve_slowdown_yaw_deg', 35.0)
        self.declare_parameter('curve_slowdown_min_factor', 0.55)

        self.turn_speed_ratio = self.get_parameter('turn_speed_ratio').value
        self.reverse_threshold = self.get_parameter('reverse_threshold').value
        self.lookahead_dist = self.get_parameter('lookahead_distance').value
        self.lookahead_max_dist = self.get_parameter('lookahead_max_distance').value
        self.lookahead_speed_gain = self.get_parameter('lookahead_speed_gain').value
        self.relocate_distance_threshold = self.get_parameter('relocate_distance_threshold').value
        self.relocate_heading_threshold = math.radians(self.get_parameter('relocate_heading_threshold_deg').value)
        self.relocate_search_window = int(self.get_parameter('relocate_search_window').value)
        self.relocate_confirm_cycles = int(self.get_parameter('relocate_confirm_cycles').value)
        self.relocate_max_jump = int(self.get_parameter('relocate_max_jump').value)
        self.reverse_soft_threshold = math.radians(self.get_parameter('reverse_soft_threshold_deg').value)
        self.reverse_soft_distance = self.get_parameter('reverse_soft_distance').value
        self.reverse_hard_distance = self.get_parameter('reverse_hard_distance').value
        self.reverse_exit_threshold = math.radians(self.get_parameter('reverse_exit_threshold_deg').value)
        self.curve_slowdown_lookahead = int(self.get_parameter('curve_slowdown_lookahead').value)
        self.curve_slowdown_yaw = math.radians(self.get_parameter('curve_slowdown_yaw_deg').value)
        self.curve_slowdown_min_factor = self.get_parameter('curve_slowdown_min_factor').value

        # State
        self.state = NavState.IDLE
        self.current_wp_idx = 0
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_yaw = 0.0
        self.current_speed = 0.0
        self.pose_received = False
        self.task_start_time = None
        self.is_reversing = False  # hysteresis flag for reverse mode
        self.relocate_bad_count = 0
        # Post-QR turnaround state
        self.qr_turnaround_last_yaw = None
        self.qr_turnaround_total_yaw = 0.0
        self.qr_turnaround_start_time = None
        self.qr_recenter_start_time = None
        self.post_qr_resume_time = None
        self.vlm_task_start_time = None
        self.vlm_capture_requested = False
        self.vlm_task_done = False
        # U-turn pre-detection: when path requires sharp doubling-back,
        # lock onto a target yaw and reverse until aligned, then jump WP idx.
        self.uturn_target_idx = None
        self.uturn_target_yaw = None  # radians
        self.drive_log_counter = 0

        # PID state
        self.lin_error_sum = 0.0
        self.lin_error_last = 0.0
        self.ang_error_sum = 0.0
        self.ang_error_last = 0.0

        self.lock = threading.Lock()

        # --- Publishers ---
        self.cmd_pub = self.create_publisher(Twist, '/nav_cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/nav_status', 10)
        self.wp_pub = self.create_publisher(Int32, '/current_waypoint', 10)
        self.get_picture_pub = self.create_publisher(Int32, '/get_picture', 10)
        # Enables/disables the vision stack (control_master routes /racing ->
        # /cmd_vel only while /vision_enable is true). /car_go kept for compat.
        self.car_go_pub = self.create_publisher(Int32, '/car_go', 10)
        self.vision_enable_pub = self.create_publisher(Bool, '/vision_enable', 10)
        self.vision_active = False

        # --- Subscribers ---
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom_combined', self.odom_callback, qos)
        self.cmd_sub = self.create_subscription(
            String, '/nav_command', self.command_callback, 10)
        # QR code result: presence of any message means a code was scanned.
        self.qr_sub = self.create_subscription(
            String, '/qrcode_result', self.qr_callback, 10)
        self.qr_scanned = False


        # Control loop timer (20 Hz)
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info(
            f'Waypoint Nav started: {len(self.waypoints)} waypoints, '
            f'reach_tol={self.reach_tol}m, auto_start={self.auto_start}')
        for i, (x, y, yaw) in enumerate(self.waypoints):
            self.get_logger().info(f'  WP[{i}]: ({x:.3f}, {y:.3f}, {math.degrees(yaw):.1f}°)')

        if self.auto_start:
            self.state = NavState.DRIVING
            self.get_logger().info('Auto-starting navigation')

    def _parse_waypoints(self, flat: list) -> List[Tuple[float, float, float]]:
        """Parse flat list [x1,y1,yaw1, x2,y2,yaw2, ...] into list of (x,y,yaw) tuples."""
        if not flat or len(flat) < 3:
            self.get_logger().warn('No valid waypoints provided!')
            return [(0.0, 0.0, 0.0)]
        wps = []
        for i in range(0, len(flat) - 2, 3):
            x, y, yaw_deg = float(flat[i]), float(flat[i+1]), float(flat[i+2])
            wps.append((x, y, math.radians(yaw_deg)))
        return wps

    def _direction_from_qr(self, raw: str) -> str:
        text = (raw or '').strip().lower()
        if text == 'anticlockwise':
            return 'counterclockwise'
        if text == 'clockwise':
            return 'clockwise'
        try:
            return 'clockwise' if int(text) % 2 == 1 else 'counterclockwise'
        except ValueError:
            self.get_logger().warn(f'Unrecognized QR route "{raw}", defaulting to clockwise')
            return 'clockwise'

    def _set_route_for_direction(self, direction: str):
        if direction != 'counterclockwise':
            self.waypoints = list(self.base_waypoints)
            self.get_logger().info(
                f'Route set to clockwise: using base waypoints ({len(self.waypoints)} points)')
            return

        n = len(self.base_waypoints)
        branch = max(0, min(self.route_branch_idx, n - 1))
        merge = max(0, min(self.route_merge_idx, n - 1))
        if branch >= merge:
            self.get_logger().warn(
                f'Invalid branch/merge ({branch}/{merge}), using base waypoints')
            self.waypoints = list(self.base_waypoints)
            self.route_direction = 'clockwise'
            return

        common = list(self.base_waypoints[:branch + 1])
        return_start = max(branch + 1, merge - 5)
        reverse_loop = [
            (x, y, self._normalize_angle(yaw + math.pi))
            for x, y, yaw in reversed(self.base_waypoints[branch + 1:return_start])
        ]
        return_to_p = list(self.base_waypoints[return_start:])
        self.waypoints = common + reverse_loop + return_to_p
        self.get_logger().info(
            f'Route set to counterclockwise: common WP[0..{branch}], '
            f'reversed WP[{branch + 1}..{return_start - 1}], return WP[{return_start}..{n - 1}] '
            f'({len(self.waypoints)} points)')

    def odom_callback(self, msg: Odometry):
        """Update current pose from EKF output."""
        q = msg.pose.pose.orientation
        yaw = _quat_to_yaw(q.x, q.y, q.z, q.w)
        with self.lock:
            self.pose_x = msg.pose.pose.position.x
            self.pose_y = msg.pose.pose.position.y
            self.pose_yaw = yaw
            self.current_speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
            self.pose_received = True

    def command_callback(self, msg: String):
        """Handle navigation commands."""
        cmd = msg.data.strip().lower()
        self.get_logger().info(
            f'NAV: command="{cmd}" state={self.state.name} vision_active={self.vision_active}')
        if cmd == 'start':
            self.qr_scanned = False
            self.vlm_task_start_time = None
            self.vlm_capture_requested = False
            self.vlm_task_done = False
            self.route_direction = 'clockwise'
            self.waypoints = list(self.base_waypoints)
            if self.lead_with_vision:
                self.get_logger().info(
                    'NAV: Start - VISION lead phase (drive to teardrop, scan QR)')
                self.state = NavState.VISION_LEAD
                self.current_wp_idx = 0
                self._activate_vision()
            else:
                self.get_logger().info('NAV: Start command received (odometry)')
                self.state = NavState.DRIVING
                self.current_wp_idx = 0
                self._deactivate_vision()
            self._reset_pid()
        elif cmd == 'pause':
            if self.state == NavState.DRIVING:
                self.state = NavState.PAUSED
                self._stop()
                self.get_logger().info('NAV: Paused')
        elif cmd == 'resume':
            if self.state == NavState.PAUSED:
                self.state = NavState.DRIVING
                self.get_logger().info('NAV: Resumed')
        elif cmd == 'reset':
            self.state = NavState.IDLE
            self.current_wp_idx = 0
            self.vlm_task_start_time = None
            self.vlm_capture_requested = False
            self.vlm_task_done = False
            self.route_direction = 'clockwise'
            self.waypoints = list(self.base_waypoints)
            self._stop()
            self._reset_pid()
            self.get_logger().info('NAV: Reset')
        elif cmd.startswith('goto:'):
            # goto:index  - jump to specific waypoint
            try:
                idx = int(cmd.split(':')[1])
                if 0 <= idx < len(self.waypoints):
                    self.current_wp_idx = idx
                    self.state = NavState.DRIVING
                    self._reset_pid()
                    self.get_logger().info(f'NAV: Goto waypoint {idx}')
            except (ValueError, IndexError):
                pass

    def qr_callback(self, msg: String):
        """QR code scanned -> stop vision, execute reverse+right-turn, then resume nearest WP."""
        self.get_logger().info(
            f'QR callback: data="{msg.data}" state={self.state.name} qr_scanned={self.qr_scanned}')
        if self.state != NavState.VISION_LEAD or self.qr_scanned:
            self.get_logger().info('QR callback ignored: not in VISION_LEAD or already scanned')
            return
        self.qr_scanned = True
        self.route_direction = self._direction_from_qr(msg.data)
        self._set_route_for_direction(self.route_direction)
        self._deactivate_vision()
        self.get_logger().info(
            f'QR transition: vision disabled, route={self.route_direction}, entering QR_TURNAROUND')
        with self.lock:
            self.qr_turnaround_last_yaw = self.pose_yaw
        self.qr_turnaround_total_yaw = 0.0
        self.qr_turnaround_start_time = self.get_clock().now()
        self.state = NavState.QR_TURNAROUND
        self._reset_pid()
        self.get_logger().info(
            f'QR scanned ("{msg.data}") -> reverse+right-turn maneuver '
            f'(target rotation {math.degrees(self.qr_yaw_threshold):.0f}°, '
            f'route={self.route_direction})')
        self._publish_status(f'QR scanned -> {self.route_direction} route -> reverse+right-turn')


    def control_loop(self):
        """Main control loop (20 Hz)."""
        if self.state == NavState.VISION_LEAD:
            # Vision drives from start until QR scanned. Keep enable alive,
            # publish NOTHING to /cmd_vel.
            self._activate_vision()
            return

        if not self.pose_received:
            return

        if self.state == NavState.QR_TURNAROUND:
            self._do_qr_turnaround()
            return

        if self.state == NavState.QR_RECENTER:
            self._do_qr_recenter()
            return

        if self.state == NavState.VLM_CAPTURE:
            self._do_vlm_capture_task()
            return


        # Publish current waypoint index
        wp_msg = Int32()
        wp_msg.data = self.current_wp_idx
        self.wp_pub.publish(wp_msg)

        with self.lock:
            px, py, pyaw = self.pose_x, self.pose_y, self.pose_yaw

        if self.state == NavState.HANDOFF:
            # Vision stack is driving now. Keep the activation signal alive and
            # publish NOTHING to /cmd_vel (avoid fighting control_master).
            self._activate_vision()
            return

        if self.state == NavState.IDLE or self.state == NavState.FINISHED or self.state == NavState.PAUSED:
            return

        # Current target waypoint
        if self.current_wp_idx >= len(self.waypoints):
            if self.loop:
                self.current_wp_idx = 0
            else:
                self.state = NavState.FINISHED
                self._stop()
                self._publish_status('FINISHED: All waypoints completed')
                return

        # --- Lookahead: advance past waypoints that are already behind/within reach ---
        self._advance_waypoint_index(px, py)
        if self.state == NavState.DRIVING:
            self._maybe_relocate_waypoint(px, py, pyaw)
            if self._maybe_start_vlm_task():
                return
        if self.current_wp_idx >= len(self.waypoints):
            if self.loop:
                self.current_wp_idx = 0
            else:
                self.state = NavState.FINISHED
                self._stop()
                self._publish_status('FINISHED: All waypoints completed')
                return

        # --- Vision handoff: switch to vision line-following at the X-crossing
        # return leg, where accumulated IMU drift makes odometry unreliable. ---
        if (self.vision_handoff_idx is not None and self.vision_handoff_idx >= 0
                and self.current_wp_idx >= self.vision_handoff_idx
                and self.state == NavState.DRIVING):
            self.get_logger().info(
                f'HANDOFF to vision line-following at WP[{self.current_wp_idx}] '
                f'(>= {self.vision_handoff_idx}); odometry nav stops here.')
            self._activate_vision()
            self.state = NavState.HANDOFF
            self._publish_status('HANDOFF: vision line-following active')
            return

        # Find lookahead target (furthest waypoint within lookahead_distance)
        target_idx = self._find_lookahead_target(px, py)
        tx, ty, t_yaw = self.waypoints[target_idx]

        # Distance and bearing to lookahead target
        dx = tx - px
        dy = ty - py
        dist = math.sqrt(dx * dx + dy * dy)
        bearing = math.atan2(dy, dx)
        heading_error = self._normalize_angle(bearing - pyaw)

        # Distance to final waypoint (for slowdown at end)
        fx, fy, _ = self.waypoints[-1]
        dist_to_final = math.sqrt((fx - px)**2 + (fy - py)**2)

        if self.state == NavState.DRIVING:
            self.drive_log_counter += 1
            if self.drive_log_counter % 20 == 0:
                lookahead = self._compute_dynamic_lookahead_distance()
                self.get_logger().info(
                    f'NAV DRIVING: wp={self.current_wp_idx} target={target_idx} '
                    f'pos=({px:.2f},{py:.2f}) dist={dist:.2f} '
                    f'herr={math.degrees(heading_error):.1f}deg '
                    f'v={self.current_speed:.2f} la={lookahead:.2f}')
            self._do_drive(heading_error, dist, dist_to_final)
        elif self.state == NavState.ARRIVED:
            self._do_task(t_yaw, pyaw)

    def _advance_waypoint_index(self, px: float, py: float):
        """
        Advance current_wp_idx sequentially.
        Skip waypoints that are:
        (a) within reach_tolerance (normal pass-through), OR
        (b) within 0.80m AND behind/beside car (bearing offset > 90°)
            AND distance > 0.20m (not too close, let normal reach handle those)
            — handles post-obstacle-avoidance lateral displacement
            without affecting normal cornering precision.
        """
        max_skip_per_cycle = 8
        skipped = 0
        lateral_skip_radius = 0.80  # m, skip waypoints behind us within this radius
        lateral_skip_angle = math.radians(90)  # truly behind/beside, not side-front
        lateral_skip_min_dist = 0.20  # m, don't skip very close waypoints

        while (self.current_wp_idx < len(self.waypoints) - 1
               and skipped < max_skip_per_cycle):
            wx, wy, _ = self.waypoints[self.current_wp_idx]
            d = math.sqrt((wx - px)**2 + (wy - py)**2)
            if d < self.reach_tol:
                self.current_wp_idx += 1
                skipped += 1
                continue
            # Lateral/behind skip: waypoint is behind/beside the car (post-avoidance)
            if d > lateral_skip_min_dist and d < lateral_skip_radius:
                bearing = math.atan2(wy - py, wx - px)
                bearing_offset = abs(self._normalize_angle(bearing - self.pose_yaw))
                if bearing_offset > lateral_skip_angle:
                    self.get_logger().info(
                        f'SKIP WP[{self.current_wp_idx}] d={d:.2f}m '
                        f'bearing={math.degrees(bearing_offset):.0f}° (behind/side)')
                    self.current_wp_idx += 1
                    skipped += 1
                    continue
            break

    def _maybe_relocate_waypoint(self, px: float, py: float, pyaw: float):
        if self.current_wp_idx >= len(self.waypoints) - 1:
            self.relocate_bad_count = 0
            return

        wx, wy, _ = self.waypoints[self.current_wp_idx]
        current_dist = math.hypot(wx - px, wy - py)
        current_bearing = math.atan2(wy - py, wx - px)
        current_heading_err = abs(self._normalize_angle(current_bearing - pyaw))
        bad_target = (current_dist > self.relocate_distance_threshold or
                      current_heading_err > self.relocate_heading_threshold)
        if not bad_target:
            self.relocate_bad_count = 0
            return

        self.relocate_bad_count += 1
        if self.relocate_bad_count < self.relocate_confirm_cycles:
            return

        end_idx = min(len(self.waypoints) - 1,
                      self.current_wp_idx + min(self.relocate_search_window, self.relocate_max_jump))
        best_idx = self.current_wp_idx
        best_score = float('inf')
        best_dist = current_dist
        best_heading = current_heading_err
        for i in range(self.current_wp_idx + 1, end_idx + 1):
            cx, cy, _ = self.waypoints[i]
            dist = math.hypot(cx - px, cy - py)
            bearing = math.atan2(cy - py, cx - px)
            heading_err = abs(self._normalize_angle(bearing - pyaw))
            if heading_err > math.radians(125.0) and dist > self.relocate_distance_threshold:
                continue
            score = dist + 0.55 * heading_err + 0.015 * (i - self.current_wp_idx)
            if score < best_score:
                best_score = score
                best_idx = i
                best_dist = dist
                best_heading = heading_err

        if best_idx > self.current_wp_idx:
            old_idx = self.current_wp_idx
            self.current_wp_idx = best_idx
            self.relocate_bad_count = 0
            self.is_reversing = False
            self._reset_pid()
            self.get_logger().info(
                f'RELOCATE WP[{old_idx}] -> WP[{best_idx}] '
                f'dist={current_dist:.2f}->{best_dist:.2f}m '
                f'herr={math.degrees(current_heading_err):.0f}->{math.degrees(best_heading):.0f}deg')
        elif self.relocate_bad_count > self.relocate_confirm_cycles * 3:
            self.relocate_bad_count = self.relocate_confirm_cycles

    def _detect_uturn_ahead(self):
        """
        Look ahead 15 waypoints. If we find a waypoint whose yaw differs from
        the current car yaw by > 100°, we have a U-turn ahead.
        Returns (exit_idx, exit_yaw_rad) or None.

        Strategy: instead of trying to follow the teardrop forward (which
        physically can't be done by Ackermann car), we will reverse with
        steering until our yaw matches the EXIT yaw, then jump to the exit
        waypoint and resume forward driving.
        """
        if self.uturn_target_idx is not None:
            return None  # already executing a U-turn
        if self.current_wp_idx + 5 >= len(self.waypoints):
            return None
        if self.current_speed > 0.18:
            return None

        car_yaw = self.pose_yaw
        look_ahead = 15
        end_idx = min(self.current_wp_idx + look_ahead, len(self.waypoints) - 1)

        # Find waypoint whose GEOMETRIC BEARING from the robot deviates most
        # from the current car heading.  This correctly catches teardrop/reverse
        # segments whose physical positions are BEHIND the car even though their
        # stored yaw values may look forward-ish (recorded while reversing).
        max_abs_diff = 0.0
        max_idx = None
        for i in range(self.current_wp_idx + 1, end_idx):
            wx, wy, _ = self.waypoints[i]
            bearing = math.atan2(wy - self.pose_y, wx - self.pose_x)
            diff = abs(self._normalize_angle(bearing - car_yaw))
            if diff > max_abs_diff:
                max_abs_diff = diff
                max_idx = i

        # Require (a) >140° bearing offset (upcoming WP is mostly behind car),
        # (b) car within 0.35m of that "behind" waypoint (we are at the apex).
        if max_idx is None or max_abs_diff < math.radians(140):
            return None
        apex_x, apex_y, _ = self.waypoints[max_idx]
        d_apex = math.hypot(apex_x - self.pose_x, apex_y - self.pose_y)
        if d_apex > 0.35:
            return None
        if max_idx > self.current_wp_idx + 8:
            return None

        current_x, current_y, _ = self.waypoints[self.current_wp_idx]
        path_seg = math.hypot(apex_x - current_x, apex_y - current_y)
        if path_seg > 0.70:
            return None

        # Find EXIT: first WP past apex where yaw RATE drops below 5deg/WP
        # (i.e. the path has finished its sharp turn and goes straight again).
        for j in range(max_idx + 1, min(max_idx + 25, len(self.waypoints) - 1)):
            y_curr = self.waypoints[j][2]      # already radians
            y_next = self.waypoints[j + 1][2]  # already radians
            yaw_rate = abs(self._normalize_angle(y_next - y_curr))
            if yaw_rate < math.radians(5):
                return j, y_curr
        # Fallback
        last_idx = min(max_idx + 10, len(self.waypoints) - 1)
        return last_idx, self.waypoints[last_idx][2]  # already radians

    def _do_qr_turnaround(self):
        """Reverse with right steering until yaw has rotated by qr_yaw_threshold,
        then find the nearest waypoint and switch to DRIVING."""
        with self.lock:
            current_yaw = self.pose_yaw
            px, py = self.pose_x, self.pose_y

        if self.qr_turnaround_start_time is not None:
            elapsed = (self.get_clock().now() - self.qr_turnaround_start_time).nanoseconds / 1e9
            if elapsed < self.qr_presteer_sec:
                self.qr_turnaround_last_yaw = current_yaw
                cmd = Twist()
                cmd.linear.x = 0.0
                cmd.angular.z = self.qr_angular   # 前轮向右转（已按实测反号）
                self.cmd_pub.publish(cmd)
                self._publish_status(f'QR_TURNAROUND pre-steer {elapsed:.1f}/{self.qr_presteer_sec:.1f}s')
                return

        # Accumulate absolute yaw change (handles wraparound)
        if self.qr_turnaround_last_yaw is not None:
            delta = abs(self._normalize_angle(current_yaw - self.qr_turnaround_last_yaw))
            self.qr_turnaround_total_yaw += delta
        self.qr_turnaround_last_yaw = current_yaw

        if self.qr_turnaround_total_yaw >= self.qr_yaw_threshold:
            self._stop()
            self.qr_recenter_start_time = self.get_clock().now()
            self.state = NavState.QR_RECENTER
            self.get_logger().info(
                f'QR turnaround reached target yaw ({math.degrees(self.qr_turnaround_total_yaw):.1f}°) '
                f'-> recenter for {self.qr_recenter_sec:.2f}s')
            self._publish_status('QR turnaround reached target yaw -> recenter steering')
            return

        # Reverse with right steering: linear.x < 0, angular.z > 0 (前轮向右，已按实测反号)
        cmd = Twist()
        cmd.linear.x = -self.qr_reverse_speed
        cmd.angular.z = self.qr_angular
        self.cmd_pub.publish(cmd)
        self._publish_status(
            f'QR_TURNAROUND rotated={math.degrees(self.qr_turnaround_total_yaw):.1f}° '
            f'/ {math.degrees(self.qr_yaw_threshold):.0f}°')

    def _do_qr_recenter(self):
        elapsed = 0.0
        if self.qr_recenter_start_time is not None:
            elapsed = (self.get_clock().now() - self.qr_recenter_start_time).nanoseconds / 1e9

        cmd = Twist()
        self.cmd_pub.publish(cmd)

        if elapsed < self.qr_recenter_sec:
            self._publish_status(f'QR_RECENTER {elapsed:.2f}/{self.qr_recenter_sec:.2f}s')
            return

        with self.lock:
            px, py, pyaw = self.pose_x, self.pose_y, self.pose_yaw
        resume_idx = self._find_resume_waypoint(px, py, pyaw)
        self.current_wp_idx = resume_idx
        self.state = NavState.DRIVING
        self.post_qr_resume_time = self.get_clock().now()
        self._reset_pid()
        self.drive_log_counter = 0
        self.get_logger().info(
            f'QR recenter done -> resume WP[{resume_idx}] pos=({px:.2f},{py:.2f}) yaw={math.degrees(pyaw):.1f}°')
        self._publish_status(f'QR recenter done -> WP[{resume_idx}]')

    def _maybe_start_vlm_task(self) -> bool:
        if not self.qr_scanned or self.vlm_task_done or self.state != NavState.DRIVING:
            return False
        target_idx = (self.clockwise_vlm_wp_idx
                      if self.route_direction == 'clockwise'
                      else self.counterclockwise_vlm_wp_idx)
        if self.current_wp_idx < target_idx:
            return False
        self.vlm_task_start_time = self.get_clock().now()
        self.vlm_capture_requested = False
        self.state = NavState.VLM_CAPTURE
        self._stop()
        self._publish_status(
            f'VLM_CAPTURE: {self.route_direction} WP[{self.current_wp_idx}], stop {self.vlm_stop_sec:.1f}s')
        self.get_logger().info(
            f'VLM task started for {self.route_direction} route at WP[{self.current_wp_idx}] '
            f'(trigger WP[{target_idx}])')
        return True

    def _do_vlm_capture_task(self):
        self._stop()
        elapsed = 0.0
        if self.vlm_task_start_time is not None:
            elapsed = (self.get_clock().now() - self.vlm_task_start_time).nanoseconds / 1e9
        if not self.vlm_capture_requested and elapsed >= self.vlm_capture_delay_sec:
            request = Int32()
            request.data = 1
            self.get_picture_pub.publish(request)
            self.vlm_capture_requested = True
            self.get_logger().info('VLM image capture requested on /get_picture')
        if elapsed < self.vlm_stop_sec:
            self._publish_status(f'VLM_CAPTURE: stopped {elapsed:.1f}/{self.vlm_stop_sec:.1f}s')
            return
        self.vlm_task_done = True
        self.vlm_task_start_time = None
        self.state = NavState.DRIVING
        self._reset_pid()
        self.get_logger().info('VLM capture stop completed; navigation resumed while recognition continues')
        self._publish_status('VLM_CAPTURE complete: navigation resumed')

    def _find_nearest_waypoint(self, px: float, py: float) -> int:
        """Return the index of the waypoint geometrically nearest to (px, py)."""
        nearest_idx = 0
        min_dist = float('inf')
        for i, (wx, wy, _) in enumerate(self.waypoints):
            d = math.hypot(wx - px, wy - py)
            if d < min_dist:
                min_dist = d
                nearest_idx = i
        self.get_logger().info(
            f'Nearest WP search: WP[{nearest_idx}] dist={min_dist:.3f}m '
            f'pos=({px:.3f},{py:.3f})')
        return nearest_idx

    def _find_resume_waypoint(self, px: float, py: float, pyaw: float) -> int:
        start_idx = max(0, min(int(self.odom_resume_idx), len(self.waypoints) - 1))
        best_idx = start_idx
        best_score = float('inf')
        target_dist = max(0.25, float(self.qr_resume_target_distance))

        for i in range(start_idx, len(self.waypoints)):
            wx, wy, _ = self.waypoints[i]
            dist = math.hypot(wx - px, wy - py)
            bearing = math.atan2(wy - py, wx - px)
            heading_err = abs(self._normalize_angle(bearing - pyaw))
            if heading_err > math.radians(130.0):
                continue
            score = abs(dist - target_dist) + 0.55 * heading_err + 0.01 * (i - start_idx)
            if score < best_score:
                best_score = score
                best_idx = i

        if best_score == float('inf'):
            best_idx = self._find_nearest_waypoint(px, py)
            self.get_logger().info(
                f'Resume WP fallback -> WP[{best_idx}] from global nearest search')
            return best_idx

        self.get_logger().info(
            f'Resume WP search: start={start_idx} -> WP[{best_idx}] score={best_score:.3f} '
            f'target_dist={target_dist:.2f} pos=({px:.3f},{py:.3f}) yaw={math.degrees(pyaw):.1f}°')
        return best_idx

    def _find_lookahead_target(self, px: float, py: float) -> int:
        """
        Find lookahead target: first waypoint beyond lookahead_distance along path.
        Limited to a search window AHEAD of current_wp_idx to avoid being
        attracted to return-trip waypoints on closed/looping paths.
        """
        # Only search a small window ahead of current waypoint
        search_window = 30  # Look at most 30 waypoints ahead (~3m of path)
        end_idx = min(self.current_wp_idx + search_window, len(self.waypoints))
        lookahead = self._compute_dynamic_lookahead_distance()
        max_bearing_offset = math.radians(100)

        target = self.current_wp_idx
        fallback_target = self.current_wp_idx
        for i in range(self.current_wp_idx, end_idx):
            wx, wy, _ = self.waypoints[i]
            d = math.sqrt((wx - px)**2 + (wy - py)**2)
            fallback_target = i
            bearing = math.atan2(wy - py, wx - px)
            bearing_offset = abs(self._normalize_angle(bearing - self.pose_yaw))
            if bearing_offset > max_bearing_offset:
                continue
            target = i
            if d >= lookahead:
                target = i
                break
        if target == self.current_wp_idx:
            target = fallback_target
        return min(target, len(self.waypoints) - 1)

    def _compute_dynamic_lookahead_distance(self) -> float:
        speed_term = self.lookahead_speed_gain * max(0.0, self.current_speed)
        return min(self.lookahead_max_dist, max(self.lookahead_dist, self.lookahead_dist + speed_term))

    def _compute_curve_speed_factor(self) -> float:
        if self.current_wp_idx >= len(self.waypoints) - 2:
            return 1.0

        end_idx = min(self.current_wp_idx + self.curve_slowdown_lookahead, len(self.waypoints) - 1)
        max_yaw_delta = 0.0
        for i in range(self.current_wp_idx, end_idx):
            yaw0 = self.waypoints[i][2]
            yaw1 = self.waypoints[i + 1][2]
            max_yaw_delta = max(max_yaw_delta, abs(self._normalize_angle(yaw1 - yaw0)))

        if max_yaw_delta <= self.curve_slowdown_yaw:
            return 1.0

        excess = min(max_yaw_delta / max(self.curve_slowdown_yaw, 1e-3), 2.0) - 1.0
        factor = 1.0 - (1.0 - self.curve_slowdown_min_factor) * excess
        return max(self.curve_slowdown_min_factor, min(1.0, factor))


    def _do_drive(self, heading_error: float, dist: float, dist_to_final: float = None):
        """
        Ackermann-compatible driving with lookahead for smooth path following.
        Only slows down near the FINAL waypoint, not intermediate ones.
        """

        # Check if we reached the FINAL waypoint
        if self.current_wp_idx >= len(self.waypoints) - 1 and dist < self.reach_tol:
            self._arrive()
            return

        abs_err = abs(heading_error)

        # === U-TURN MODE (pre-emptive 3-point turn for sharp teardrops) ===
        # If we're currently executing a U-turn, keep reversing until yaw aligns.
        if self.uturn_target_yaw is not None:
            yaw_err = self._normalize_angle(self.uturn_target_yaw - self.pose_yaw)
            # Tight exit threshold (8\u00b0) so car completes the full rotation,
            # not just gets close. Otherwise next forward step swings wrong way.
            if abs(yaw_err) < math.radians(8):
                self.get_logger().info(
                    f'U-turn DONE: jumping WP[{self.current_wp_idx}] -> '
                    f'WP[{self.uturn_target_idx}] '
                    f'(yaw {math.degrees(self.pose_yaw):.1f}\u00b0 '
                    f'target {math.degrees(self.uturn_target_yaw):.1f}\u00b0)')
                self.current_wp_idx = self.uturn_target_idx
                self.uturn_target_idx = None
                self.uturn_target_yaw = None
                self.is_reversing = False
                return  # next cycle resumes forward driving
            # Continue reversing: steer such that car rotates toward target yaw.
            # In reverse, positive w (CCW yaw rate) requires right-steer.
            w = self.max_w if yaw_err > 0 else -self.max_w
            # Use 2x min_v for reverse - too slow rotation otherwise
            v = -max(self.min_v * 2.0, 0.20)
            cmd = Twist()
            cmd.linear.x = v
            cmd.angular.z = w
            self.cmd_pub.publish(cmd)
            self._publish_status(
                f'UTURN -> WP[{self.uturn_target_idx}] '
                f'yaw_err={math.degrees(yaw_err):+.1f}\u00b0 v={v:.2f}')
            return

        # Otherwise check if a U-turn lies ahead and enter U-turn mode.
        uturn = self._detect_uturn_ahead()
        if uturn is not None:
            exit_idx, exit_yaw = uturn
            self.uturn_target_idx = exit_idx
            self.uturn_target_yaw = exit_yaw
            self.is_reversing = True
            self.get_logger().info(
                f'U-TURN detected at WP[{self.current_wp_idx}]! '
                f'car_yaw={math.degrees(self.pose_yaw):.1f}\u00b0 -> '
                f'target_yaw={math.degrees(exit_yaw):.1f}\u00b0 '
                f'skipping to WP[{exit_idx}]')
            return  # next cycle will execute U-turn logic above

        # === Reverse mode with hysteresis (reactive, for unexpected overshoot) ===
        # Enter reverse when heading error exceeds reverse_threshold (from config).
        # Exit reverse only when heading error drops below 80deg (hysteresis).
        enter_reverse = self.reverse_threshold
        exit_reverse = math.radians(80)

        reverse_locked = False
        if self.post_qr_resume_time is not None:
            elapsed_since_resume = (self.get_clock().now() - self.post_qr_resume_time).nanoseconds / 1e9
            reverse_locked = elapsed_since_resume < self.qr_reverse_lockout_sec

        soft_reverse = dist < self.reverse_soft_distance and abs_err > self.reverse_soft_threshold
        hard_reverse = dist < self.reverse_hard_distance and abs_err > enter_reverse
        if not self.is_reversing and not reverse_locked and (soft_reverse or hard_reverse):
            self.is_reversing = True
            level = 'hard' if hard_reverse else 'soft'
            self.get_logger().info(
                f'Enter REVERSE({level}): WP[{self.current_wp_idx}] dist={dist:.2f}m '
                f'herr={math.degrees(heading_error):.1f}°')
        elif self.is_reversing and abs_err < self.reverse_exit_threshold:
            self.is_reversing = False
            self.get_logger().info(
                f'Exit REVERSE: WP[{self.current_wp_idx}] herr={math.degrees(heading_error):.1f}°')

        if reverse_locked and self.is_reversing:
            self.is_reversing = False

        if reverse_locked and self.drive_log_counter % 20 == 0:
            remaining = max(0.0, self.qr_reverse_lockout_sec - elapsed_since_resume)
            self.get_logger().info(
                f'QR reverse lockout active: {remaining:.2f}s remaining at WP[{self.current_wp_idx}]')

        if self.is_reversing:
            # Reversing: target is behind, drive backward with steering that
            # swings the car's front toward the target.
            # Steering: w sign matches heading_error sign (front swings toward target).
            w = self.max_w if heading_error > 0 else -self.max_w
            v = -self.min_v  # slow reverse

            cmd = Twist()
            cmd.linear.x = v
            cmd.angular.z = w
            self.cmd_pub.publish(cmd)
            self._publish_status(
                f'REVERSE WP[{self.current_wp_idx}] dist={dist:.3f}m '
                f'herr={math.degrees(heading_error):.1f}° v={v:.2f}')
            return

        # Speed factor based on heading error (progressive: earlier and stronger slowdown)
        if abs_err < self.angle_tol:
            speed_factor = 1.0
        else:
            max_steer_angle = math.radians(90)
            t = min(abs_err / max_steer_angle, 1.0)
            # Square-root curve: slowdown starts early but leaves room at extreme
            speed_factor = 1.0 - (1.0 - self.turn_speed_ratio) * (t ** 0.5)

        # Only slow down near the FINAL waypoint
        if dist_to_final is not None and dist_to_final < self.slowdown_radius:
            v_target = self.min_v + (self.max_v - self.min_v) * (dist_to_final / self.slowdown_radius)
        else:
            v_target = self.max_v

        v_target *= self._compute_curve_speed_factor()

        if self.post_qr_resume_time is not None:
            elapsed_since_resume = (self.get_clock().now() - self.post_qr_resume_time).nanoseconds / 1e9
            if elapsed_since_resume < self.qr_resume_speed_cap_sec:
                v_target = min(v_target, self.qr_resume_speed_cap)

        v = v_target * speed_factor
        v = max(self.min_v, min(v, self.max_v))

        # Angular velocity from PID
        w = self._pid_angular(heading_error)

        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)

        self._publish_status(
            f'DRIVING WP[{self.current_wp_idx}] dist={dist:.3f}m '
            f'herr={math.degrees(heading_error):.1f}° v={v:.2f}')

    def _do_task(self, target_yaw: float, current_yaw: float):
        """Execute task at waypoint (pause for task_duration)."""
        now = self.get_clock().now()
        if self.task_start_time is None:
            self.task_start_time = now
            self._stop()
            self._publish_status(
                f'ARRIVED WP[{self.current_wp_idx}], executing task...')
            self.get_logger().info(
                f'Arrived at WP[{self.current_wp_idx}], task for {self.task_duration}s')
            return

        elapsed = (now - self.task_start_time).nanoseconds / 1e9
        if elapsed >= self.task_duration:
            # Task done, move to next waypoint
            self.task_start_time = None
            self.current_wp_idx += 1
            self.state = NavState.DRIVING
            self._reset_pid()
            self.get_logger().info(
                f'Task done, moving to WP[{self.current_wp_idx}]')

    def _arrive(self):
        """Mark arrival at current waypoint (only final waypoint triggers task)."""
        self._stop()
        if self.task_duration > 0:
            self.state = NavState.ARRIVED
            self.task_start_time = None
        else:
            # No task, immediately finish
            self.state = NavState.FINISHED
            self._publish_status('FINISHED: All waypoints completed')
            self.get_logger().info('Path complete!')
        self._reset_pid()

    def _pid_linear(self, dist: float, v_target: float) -> float:
        """PID for linear velocity."""
        error = dist
        self.lin_error_sum += error
        d_error = error - self.lin_error_last
        self.lin_error_last = error

        v = self.lin_kp * error + self.lin_ki * self.lin_error_sum + self.lin_kd * d_error
        v = max(self.min_v, min(v, v_target))
        return v

    def _pid_angular(self, heading_error: float) -> float:
        """PID for angular velocity."""
        error = heading_error
        self.ang_error_sum += error
        # Anti-windup
        self.ang_error_sum = max(-1.0, min(1.0, self.ang_error_sum))
        d_error = error - self.ang_error_last
        self.ang_error_last = error

        w = self.ang_kp * error + self.ang_ki * self.ang_error_sum + self.ang_kd * d_error
        w = max(-self.max_w, min(self.max_w, w))
        return w

    def _reset_pid(self):
        """Reset PID accumulators."""
        self.lin_error_sum = 0.0
        self.lin_error_last = 0.0
        self.ang_error_sum = 0.0
        self.ang_error_last = 0.0

    def _stop(self):
        """Publish zero velocity (no-op if context already shut down)."""
        if not rclpy.ok():
            return
        cmd = Twist()
        try:
            self.cmd_pub.publish(cmd)
        except Exception:
            pass

    def _activate_vision(self):
        """Enable the vision stack to drive /cmd_vel."""
        en = Bool()
        en.data = True
        self.vision_enable_pub.publish(en)
        go = Int32()
        go.data = -10
        self.car_go_pub.publish(go)
        if not self.vision_active:
            self.vision_active = True
            self.get_logger().info('Vision stack ENABLED (/vision_enable = true)')

    def _deactivate_vision(self):
        """Silence the vision stack so odometry nav can drive /cmd_vel."""
        en = Bool()
        en.data = False
        self.vision_enable_pub.publish(en)
        if self.vision_active:
            self.vision_active = False
            self.get_logger().info('Vision stack DISABLED (/vision_enable = false)')

    def _publish_status(self, text: str):
        """Publish navigation status."""
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def destroy_node(self):
        self._stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node._stop()          # publish zero-vel BEFORE context is torn down
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
