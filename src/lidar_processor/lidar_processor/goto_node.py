"""Fly the simulated drone to a goal through PX4 Offboard mode.

Goal: ros2 topic pub --once /goto/goal geometry_msgs/msg/Point \
          "{x: 5.0, y: 0.0, z: 2.0}"
x = North (m), y = East (m) in PX4's local frame (origin = spawn point),
z = height above ground (m, up). z <= 0 uses the default takeoff height.
RViz: Fixed Frame "map" (ENU, origin = spawn point), press "2D Goal Pose" and
click; x = East, y = North. The drone is published on /goto/drone_pose.
Land: ros2 topic pub --once /goto/land std_msgs/msg/Empty "{}"

Obstacles: each tick the node checks a drone-wide corridor in candidate
directions from the LiDAR and flies the free one closest to the goal
direction. When the direct way is blocked it commits to one side and follows
the obstacle until the goal direction is clear with extra margin, so it can
get round a long wall. It hovers when the scan is stale or the committed side
is blocked, and gives the side up after `detour_release_s`. It is reactive
(no map), so a U-shaped trap can still defeat it.
"""
import math
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Point, PoseStamped
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Empty
from visualization_msgs.msg import Marker, MarkerArray
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)

NAN = float('nan')

IDLE, ARMING, TAKEOFF, NAVIGATE, HOLD, LANDING = range(6)
STATE_NAMES = ['IDLE', 'ARMING', 'TAKEOFF', 'NAVIGATE', 'HOLD', 'LANDING']


def wrap_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def enu_to_ned(x_east, y_north):
    return y_north, x_east


def ned_yaw_to_enu_quaternion(heading):
    """Quaternion (z, w) for a NED yaw (clockwise from North) in ENU."""
    yaw = math.pi / 2.0 - heading
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def scan_to_enu(scan, east, north, heading, max_range, stride=2):
    """Valid LiDAR returns as (east, north, range) in the map (ENU) frame.

    heading is PX4's NED yaw (clockwise from North). LiDAR angle is
    counter-clockwise (left positive)."""
    sin_h, cos_h = math.sin(heading), math.cos(heading)
    points = []
    for i in range(0, len(scan.ranges), stride):
        r = scan.ranges[i]
        if not math.isfinite(r) or r < scan.range_min or r > max_range:
            continue
        angle = scan.angle_min + i * scan.angle_increment
        fwd, left = r * math.cos(angle), r * math.sin(angle)
        points.append((
            east + fwd * sin_h - left * cos_h,
            north + fwd * cos_h + left * sin_h,
            r,
        ))
    return points


def make_cloud(stamp, points, z=0.0):
    """PointCloud2 in the map frame from (east, north) pairs."""
    msg = PointCloud2()
    msg.header.stamp = stamp
    msg.header.frame_id = 'map'
    msg.height = 1
    msg.width = len(points)
    msg.fields = [
        PointField(name=n, offset=o, datatype=PointField.FLOAT32, count=1)
        for n, o in (('x', 0), ('y', 4), ('z', 8))
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * len(points)
    msg.is_dense = True
    flat = []
    for p in points:
        flat.extend((p[0], p[1], z))
    msg.data = struct.pack('<%df' % len(flat), *flat)
    return msg


def carrot(north, east, goal_north, goal_east, lead):
    """Point up to `lead` metres from (north, east) toward the goal."""
    d_n = goal_north - north
    d_e = goal_east - east
    dist = math.hypot(d_n, d_e)
    if dist <= lead or dist < 1e-6:
        return goal_north, goal_east, dist
    scale = lead / dist
    return north + d_n * scale, east + d_e * scale, dist


def near_returns(scan, length):
    """(range, angle) of valid returns closer than `length`."""
    out = []
    for i, r in enumerate(scan.ranges):
        if math.isfinite(r) and scan.range_min <= r < length:
            out.append((r, scan.angle_min + i * scan.angle_increment))
    return out


def corridor_blocked(returns, ros_angle, half_width, length):
    """True if any return lies inside the corridor of `half_width` and
    `length` pointing along ros_angle (LiDAR angle, left positive)."""
    for r, angle in returns:
        rel = angle - ros_angle
        forward = r * math.cos(rel)
        if 0.0 < forward < length and abs(r * math.sin(rel)) < half_width:
            return True
    return False


def choose_direction(scan, ros_target, half_width, length, step, fov,
                     side_lock):
    """Free LiDAR-frame direction closest to ros_target, or None.

    Candidates lie within +-fov of straight ahead. side_lock (+1 left of the
    target, -1 right, 0 none) restricts candidates to that side of the
    target, so a detour does not flip-flop."""
    returns = near_returns(scan, length)
    candidates = []
    if abs(ros_target) <= fov:
        candidates.append(ros_target)
    k = 1
    while k * step <= fov:
        candidates.extend([k * step, -k * step])
        k += 1
    best = None
    best_dev = None
    for c in candidates:
        dev = wrap_pi(c - ros_target)
        side = (dev > 1e-6) - (dev < -1e-6)
        if side_lock and side != side_lock:
            continue
        if corridor_blocked(returns, c, half_width, length):
            continue
        if best_dev is None or abs(dev) < best_dev:
            best, best_dev = c, abs(dev)
    return best


class GotoNode(Node):

    def __init__(self):
        super().__init__('goto_node')

        self.declare_parameter('takeoff_altitude', 2.0)
        self.declare_parameter('lead_distance', 1.0)
        self.declare_parameter('arrival_radius', 0.3)
        self.declare_parameter('stop_distance', 2.0)
        self.declare_parameter('drone_radius', 0.386)
        self.declare_parameter('safety_margin', 0.15)
        self.declare_parameter('candidate_step_deg', 10.0)
        self.declare_parameter('search_fov_deg', 100.0)
        self.declare_parameter('detour_exit_margin', 0.4)
        self.declare_parameter('detour_release_s', 6.0)
        self.declare_parameter('visual_range', 12.0)
        self.declare_parameter('scan_timeout', 1.0)
        self.declare_parameter('scan_topic', '/scan')

        self.takeoff_alt = self.get_parameter('takeoff_altitude').value
        self.lead = self.get_parameter('lead_distance').value
        self.arrival = self.get_parameter('arrival_radius').value
        self.stop_dist = self.get_parameter('stop_distance').value
        self.half_width = (
            self.get_parameter('drone_radius').value
            + self.get_parameter('safety_margin').value)
        self.step = math.radians(
            self.get_parameter('candidate_step_deg').value)
        self.fov = math.radians(self.get_parameter('search_fov_deg').value)
        self.exit_margin = self.get_parameter('detour_exit_margin').value
        self.release_s = self.get_parameter('detour_release_s').value
        self.visual_range = self.get_parameter('visual_range').value
        self.scan_timeout = self.get_parameter('scan_timeout').value

        self.state = IDLE
        self.goal = None
        self.goal_alt = self.takeoff_alt
        self.pos = None
        self.pos_time = None
        self.status = None
        self.scan = None
        self.scan_time = None
        self.hold_n = self.hold_e = None
        self.yaw_sp = 0.0
        self.stream_count = 0
        self.last_cmd_time = 0.0
        self.not_offboard_since = None
        self.blocked = False
        self.avoid_side = 0
        self.arm_attempts = 0
        self.travel = None
        self.visual_tick = 0
        self.seen = set()
        self.blocked_since = None
        self.last_side = 0
        self.last_exit_time = -1e9
        self.last_blocked_log = 0.0

        px4_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self.on_position, px4_qos)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v5',
            self.on_status, px4_qos)
        self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value,
            self.on_scan, 10)
        self.create_subscription(Point, '/goto/goal', self.on_goal, 10)
        self.create_subscription(
            PoseStamped, '/goal_pose', self.on_goal_pose, 10)
        self.pose_pub = self.create_publisher(
            PoseStamped, '/goto/drone_pose', 10)
        self.create_subscription(Empty, '/goto/land', self.on_land, 10)

        self.mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command_v1', 10)

        self.scan_pub = self.create_publisher(
            PointCloud2, '/goto/scan_points', 1)
        self.near_pub = self.create_publisher(
            PointCloud2, '/goto/near_points', 1)
        self.map_pub = self.create_publisher(
            PointCloud2, '/goto/map_points', 1)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/goto/markers', 1)

        self.create_timer(0.1, self.tick)
        self.get_logger().info('goto: ready, waiting for /goto/goal')

    # ---------------- inputs ----------------

    def now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_position(self, msg):
        self.pos = msg
        self.pos_time = self.now_s()

    def on_status(self, msg):
        self.status = msg

    def on_scan(self, msg):
        self.scan = msg
        self.scan_time = self.now_s()

    def on_goal_pose(self, msg):
        if msg.header.frame_id not in ('map', ''):
            self.get_logger().warn(
                f'goto: /goal_pose frame "{msg.header.frame_id}" ignored; '
                'set the RViz Fixed Frame to "map"')
            return
        north, east = enu_to_ned(msg.pose.position.x, msg.pose.position.y)
        height = self.goal_alt if self.goal is not None else self.takeoff_alt
        self.on_goal(Point(x=north, y=east, z=height))

    def on_goal(self, msg):
        if self.state == LANDING:
            self.get_logger().warn('goto: goal ignored while landing')
            return
        self.goal = (msg.x, msg.y)
        self.goal_alt = msg.z if msg.z > 0.0 else self.takeoff_alt
        self.get_logger().info(
            f'goto: goal North={msg.x:.2f} East={msg.y:.2f} '
            f'height={self.goal_alt:.2f}')
        if self.state in (IDLE, HOLD):
            self.set_state(ARMING if not self.is_airborne() else NAVIGATE)

    def on_land(self, _msg):
        self.get_logger().info('goto: land requested')
        self.send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.goal = None
        self.set_state(LANDING)

    # ---------------- helpers ----------------

    def set_state(self, state):
        if state != self.state:
            self.get_logger().info(
                f'goto: {STATE_NAMES[self.state]} -> {STATE_NAMES[state]}')
            self.state = state
            self.stream_count = 0
            self.arm_attempts = 0
            self.not_offboard_since = None

    def position_ok(self):
        return (
            self.pos is not None
            and self.pos.xy_valid and self.pos.z_valid
            and self.now_s() - self.pos_time < 1.0
        )

    def is_armed(self):
        return (
            self.status is not None
            and self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED
        )

    def is_offboard(self):
        return (
            self.status is not None
            and self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        )

    def is_airborne(self):
        return self.is_armed() and self.pos is not None and self.pos.z < -0.5

    def timestamp(self):
        return int(self.get_clock().now().nanoseconds // 1000)

    def send_command(self, command, p1=0.0, p2=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.timestamp()
        msg.command = command
        msg.param1 = float(p1)
        msg.param2 = float(p2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)

    def publish_setpoint(self, north, east, down, yaw):
        mode = OffboardControlMode()
        mode.timestamp = self.timestamp()
        mode.position = True
        self.mode_pub.publish(mode)

        sp = TrajectorySetpoint()
        sp.timestamp = self.timestamp()
        sp.position = [float(north), float(east), float(down)]
        sp.velocity = [NAN, NAN, NAN]
        sp.acceleration = [NAN, NAN, NAN]
        sp.yaw = float(yaw)
        sp.yawspeed = NAN
        self.setpoint_pub.publish(sp)

    def publish_drone_pose(self, north, east, down):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = east
        msg.pose.position.y = north
        msg.pose.position.z = -down
        msg.pose.orientation.z, msg.pose.orientation.w = \
            ned_yaw_to_enu_quaternion(self.pos.heading)
        self.pose_pub.publish(msg)

    def publish_visuals(self, north, east):
        stamp = self.get_clock().now().to_msg()
        fresh = (
            self.scan is not None
            and self.now_s() - self.scan_time <= self.scan_timeout)
        if fresh:
            pts = scan_to_enu(
                self.scan, east, north, self.pos.heading, self.visual_range)
            self.scan_pub.publish(make_cloud(stamp, pts, 0.05))
            self.near_pub.publish(make_cloud(
                stamp, [p for p in pts if p[2] < self.stop_dist], 0.1))
            if len(self.seen) < 40000:
                for p in pts:
                    self.seen.add((round(p[0] / 0.2), round(p[1] / 0.2)))
            if self.visual_tick % 10 == 0:
                self.map_pub.publish(make_cloud(
                    stamp, [(i * 0.2, j * 0.2) for i, j in self.seen], 0.0))

        markers = MarkerArray()
        goal = Marker()
        goal.header.stamp = stamp
        goal.header.frame_id = 'map'
        goal.ns = 'goto'
        goal.id = 0
        goal.type = Marker.SPHERE
        if self.goal is None or self.state in (IDLE, LANDING):
            goal.action = Marker.DELETE
        else:
            goal.action = Marker.ADD
            goal.pose.position.x = self.goal[1]
            goal.pose.position.y = self.goal[0]
            goal.pose.position.z = 0.3
            goal.pose.orientation.w = 1.0
            goal.scale.x = goal.scale.y = goal.scale.z = 0.5
            goal.color.r, goal.color.g, goal.color.a = 1.0, 0.9, 1.0
        markers.markers.append(goal)

        arrow = Marker()
        arrow.header.stamp = stamp
        arrow.header.frame_id = 'map'
        arrow.ns = 'goto'
        arrow.id = 1
        arrow.type = Marker.ARROW
        if self.travel is None or self.state != NAVIGATE:
            arrow.action = Marker.DELETE
        else:
            bearing, mode = self.travel
            arrow.action = Marker.ADD
            start, end = Point(), Point()
            start.x, start.y, start.z = east, north, 0.3
            end.x = east + 2.0 * math.sin(bearing)
            end.y = north + 2.0 * math.cos(bearing)
            end.z = 0.3
            arrow.points = [start, end]
            arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.1, 0.25, 0.3
            arrow.color.a = 1.0
            if mode == 'direct':
                arrow.color.g = 1.0
            else:
                arrow.color.r, arrow.color.g = 1.0, 0.5
        markers.markers.append(arrow)
        self.marker_pub.publish(markers)

    # ---------------- main loop ----------------

    def tick(self):
        if not self.position_ok():
            return

        north, east, down = self.pos.x, self.pos.y, self.pos.z
        self.publish_drone_pose(north, east, down)
        self.visual_tick += 1
        if self.visual_tick % 2 == 0:
            self.publish_visuals(north, east)

        if self.state in (IDLE, LANDING):
            self.yaw_sp = self.pos.heading
            self.publish_setpoint(north, east, down, self.yaw_sp)
            if self.state == LANDING and not self.is_armed():
                self.set_state(IDLE)
            return

        if self.state == ARMING:
            self.publish_setpoint(north, east, down, self.pos.heading)
            self.stream_count += 1
            if self.is_armed() and self.is_offboard():
                self.hold_n, self.hold_e = north, east
                self.yaw_sp = self.pos.heading
                self.set_state(TAKEOFF)
            elif self.stream_count >= 10 and self.now_s() - self.last_cmd_time > 1.0:
                self.last_cmd_time = self.now_s()
                self.send_command(
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                self.send_command(
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                self.arm_attempts += 1
                if self.arm_attempts in (1, 5) or self.arm_attempts % 20 == 0:
                    self.get_logger().warn(
                        f'goto: asking PX4 to arm (attempt '
                        f'{self.arm_attempts}); if it never arms, PX4 is '
                        'refusing: open QGroundControl (the "No connection '
                        'to the GCS" check) and look at its message')
            return

        if self.is_armed() and not self.is_offboard():
            now = self.now_s()
            if self.not_offboard_since is None:
                self.not_offboard_since = now
            elif now - self.not_offboard_since > 1.0:
                self.get_logger().error(
                    'goto: left Offboard mode (failsafe or another mode '
                    'requested); stopping, goal cleared')
                self.goal = None
                self.set_state(IDLE)
                return
        else:
            self.not_offboard_since = None

        if not self.is_armed():
            self.get_logger().error('goto: disarmed in flight; stopping')
            self.goal = None
            self.set_state(IDLE)
            return

        target_down = -self.goal_alt

        if self.state == TAKEOFF:
            self.publish_setpoint(
                self.hold_n, self.hold_e, target_down, self.yaw_sp)
            if abs(down - target_down) < 0.3:
                self.set_state(NAVIGATE)
            return

        if self.goal is None:
            self.publish_setpoint(north, east, target_down, self.pos.heading)
            return

        goal_n, goal_e = self.goal
        lead_n, lead_e, dist = carrot(north, east, goal_n, goal_e, self.lead)

        if self.state == HOLD:
            self.publish_setpoint(goal_n, goal_e, target_down, self.yaw_sp)
            return

        if dist < self.arrival:
            self.get_logger().info(
                f'goto: arrived (North={north:.2f} East={east:.2f})')
            self.set_state(HOLD)
            return

        bearing = math.atan2(goal_e - east, goal_n - north)
        if dist > 1.0:
            self.yaw_sp = bearing

        if self.scan is None or self.now_s() - self.scan_time > self.scan_timeout:
            self.hover_blocked('scan stale', north, east, target_down)
            return

        # NED yaw is clockwise; LiDAR angle is counter-clockwise (FLU).
        ros_target = -wrap_pi(bearing - self.pos.heading)
        length = min(self.stop_dist, max(dist, 0.5))

        if self.avoid_side:
            returns = near_returns(self.scan, length)
            if not corridor_blocked(
                    returns, ros_target,
                    self.half_width + self.exit_margin, length):
                self.get_logger().info(
                    'goto: goal direction clear, leaving detour')
                self.last_side = self.avoid_side
                self.last_exit_time = self.now_s()
                self.avoid_side = 0

        def choose(lock):
            return choose_direction(
                self.scan, ros_target, self.half_width, length, self.step,
                self.fov, lock)

        choice = choose(self.avoid_side)
        if self.avoid_side == 0 and choice is not None:
            deviation = wrap_pi(choice - ros_target)
            if (abs(deviation) > 1e-6 and self.last_side
                    and self.now_s() - self.last_exit_time < 5.0):
                locked = choose(self.last_side)
                if locked is not None:
                    choice = locked

        if choice is None:
            if self.avoid_side and self.blocked_since is not None and \
                    self.now_s() - self.blocked_since > self.release_s:
                self.get_logger().warn(
                    'goto: committed side blocked, trying the other side')
                self.avoid_side = 0
                self.blocked_since = None
            self.hover_blocked(
                'no free direction', north, east, target_down)
            return

        if self.blocked:
            self.get_logger().info('goto: path found, resuming')
            self.blocked = False
            self.blocked_since = None

        deviation = wrap_pi(choice - ros_target)
        if abs(deviation) < 1e-6:
            self.travel = (bearing, 'direct')
            self.publish_setpoint(lead_n, lead_e, target_down, self.yaw_sp)
            return

        side = 1 if deviation > 0 else -1
        if side != self.avoid_side:
            self.get_logger().info(
                f'goto: obstacle ahead, committing to detour '
                f'{"left" if side > 0 else "right"} of the goal direction '
                f'({math.degrees(deviation):+.0f} deg)')
        self.avoid_side = side
        travel = self.pos.heading - choice
        self.travel = (travel, 'detour')
        self.yaw_sp = travel
        self.publish_setpoint(
            north + self.lead * math.cos(travel),
            east + self.lead * math.sin(travel),
            target_down, self.yaw_sp)

    def hover_blocked(self, reason, north, east, target_down):
        self.travel = None
        if not self.blocked or self.now_s() - self.last_blocked_log > 5.0:
            self.get_logger().warn(f'goto: holding position, {reason}')
            self.last_blocked_log = self.now_s()
        if not self.blocked:
            self.blocked_since = self.now_s()
        self.blocked = True
        self.publish_setpoint(north, east, target_down, self.yaw_sp)


def main(args=None):
    rclpy.init(args=args)
    node = GotoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
