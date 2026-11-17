import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import LaserScan
from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude

from lidar_processor.path_planner import PathPlanner
from lidar_processor.coordinate_transform import CoordinateTransform


class AvoidanceController(Node):

    def __init__(self):
        super().__init__('avoidance_controller')

        # Sensor data
        self.scan = None

        self.scan_time = None
        self.position_time = None
        self.attitude_time = None

        self.scan_received = False
        self.position_received = False
        self.attitude_received = False
        self.heading_ready = False

        # PX4 local NED position and heading
        self.local_north = None
        self.local_east = None
        self.heading = None

        # Read-only target projection distance
        self.projection_distance = 1.0

        # Planner and coordinate transformation
        self.planner = PathPlanner()
        self.transform = CoordinateTransform()

        # PX4 QoS
        px4_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        # LiDAR subscription
        self.create_subscription(
            LaserScan,
            '/world/walls/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan',
            self.scan_callback,
            10
        )

        # PX4 position subscription
        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.position_callback,
            px4_qos
        )

        # PX4 attitude subscription
        self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.attitude_callback,
            px4_qos
        )

        # Planning loop
        self.create_timer(1.0, self.safety_check)

        self.get_logger().info(
            'INTELLIGENT PATH PLANNER STARTED'
        )

        self.get_logger().info(
            'MODE: READ ONLY - NO FLIGHT COMMANDS'
        )

    # --------------------------------------------------
    # LiDAR callback
    # --------------------------------------------------

    def scan_callback(self, msg):

        self.scan = msg

        metadata_valid = (
            len(msg.ranges) > 0
            and math.isfinite(msg.angle_min)
            and math.isfinite(msg.angle_max)
            and math.isfinite(msg.angle_increment)
            and math.isfinite(msg.range_min)
            and math.isfinite(msg.range_max)
            and msg.angle_increment > 0.0
            and msg.range_min >= 0.0
            and msg.range_max > msg.range_min
            and msg.angle_max >= msg.angle_min
        )

        self.scan_received = metadata_valid
        self.scan_time = self.get_clock().now()

    # --------------------------------------------------
    # PX4 position callback
    # --------------------------------------------------

    def position_callback(self, msg):

        position_values = [
            msg.x, msg.y, msg.z,
            msg.vx, msg.vy, msg.vz
        ]

        position_finite = all(
            math.isfinite(v) for v in position_values
        )

        heading_finite = math.isfinite(msg.heading)

        self.position_received = (
            msg.xy_valid
            and msg.z_valid
            and msg.v_xy_valid
            and msg.v_z_valid
            and not msg.dead_reckoning
            and position_finite
        )

        self.heading_ready = (
            msg.heading_good_for_control
            and heading_finite
        )

        if position_finite and heading_finite:
            self.local_north = msg.x
            self.local_east = msg.y
            self.heading = msg.heading

        self.position_time = self.get_clock().now()

    # --------------------------------------------------
    # PX4 attitude callback
    # --------------------------------------------------

    def attitude_callback(self, msg):

        q = msg.q

        valid = (
            len(q) == 4
            and all(math.isfinite(v) for v in q)
        )

        if valid:
            norm = math.sqrt(sum(v * v for v in q))
            valid = 0.9 <= norm <= 1.1

        self.attitude_received = valid
        self.attitude_time = self.get_clock().now()

    # --------------------------------------------------
    # Sensor freshness check
    # --------------------------------------------------

    def is_fresh(self, timestamp, now):

        if timestamp is None:
            return False

        age = (now - timestamp).nanoseconds

        return 0 <= age < 1_000_000_000

    # --------------------------------------------------
    # Read-only target projection
    # --------------------------------------------------

    def calculate_target(self, lidar_direction):

        if (
            self.local_north is None
            or self.local_east is None
            or self.heading is None
        ):
            return None

        # Update using live PX4 heading
        self.transform.update_yaw(self.heading)

        # ROS LiDAR FLU to PX4 body FRD
        px4_body_angle = -lidar_direction

        target_north, target_east = (
            self.transform.calculate_target(
                self.local_north,
                self.local_east,
                px4_body_angle,
                self.projection_distance
            )
        )

        return target_north, target_east

    # --------------------------------------------------
    # Main safety and planning loop
    # --------------------------------------------------

    def safety_check(self):

        now = self.get_clock().now()

        lidar_ok = (
            self.scan_received
            and self.is_fresh(self.scan_time, now)
        )

        position_ok = (
            self.position_received
            and self.is_fresh(self.position_time, now)
        )

        attitude_ok = (
            self.attitude_received
            and self.is_fresh(self.attitude_time, now)
        )

        if not lidar_ok:

            self.get_logger().warning(
                'SAFETY STOP | LiDAR missing, invalid or stale'
            )

            return

        if not position_ok or not attitude_ok:

            self.get_logger().warning(
                'DIAGNOSTICS | '
                f'Position: {position_ok} | '
                f'Attitude: {attitude_ok}'
            )

        # Analyse LiDAR
        try:

            sectors = self.planner.analyse_scan(self.scan)
            result = self.planner.plan(sectors)

        except Exception as error:

            self.get_logger().error(
                f'SAFETY STOP | Planner error: {error}'
            )

            return

        direction = result.get('direction')
        speed = result.get('speed')
        decision = result.get('decision', 'UNKNOWN')

        if direction is None:

            self.get_logger().warning(
                f'{decision} | NO TARGET GENERATED'
            )

            return

        # Calculate a diagnostic target only
        if (
            position_ok
            and attitude_ok
            and self.heading is not None
        ):

            target = self.calculate_target(direction)

            if target is not None:

                north, east = target

                heading_status = (
                    'HEADING READY'
                    if self.heading_ready
                    else 'HEADING NOT APPROVED FOR CONTROL'
                )

                self.get_logger().info(
                    f'TARGET NED (m): '
                    f'North={north:.3f}, East={east:.3f} | '
                    f'{heading_status} | '
                    f'DIAGNOSTIC ONLY'
                )

        else:

            self.get_logger().warning(
                'TARGET NOT GENERATED | '
                'Position or attitude unavailable'
            )

        self.get_logger().info(
            f'LiDAR DIRECTION: {direction:.1f} degrees | '
            f'SPEED SUGGESTION: {speed:.2f} planner units | '
            f'DECISION: {decision} | '
            f'MODE: READ ONLY'
        )


# ------------------------------------------------------
# Main function
# ------------------------------------------------------

def main(args=None):

    rclpy.init(args=args)

    node = AvoidanceController()

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
