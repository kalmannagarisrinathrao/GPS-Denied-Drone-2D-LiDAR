import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

from lidar_processor.obstacle_avoidance import obstacle_avoidance


class LidarProcessor(Node):

    def __init__(self):
        super().__init__('lidar_processor')

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )

        self.get_logger().info('LiDAR Obstacle Avoidance Started')
        self.last_log_time = self.get_clock().now()

    def get_min_distance(self, ranges, angle_min, angle_increment,
                         start_angle, end_angle, range_min, range_max):

        distances = []

        for i, distance in enumerate(ranges):
            angle = angle_min + i * angle_increment
            angle_deg = math.degrees(angle)

            if start_angle <= angle_deg <= end_angle:
                if math.isfinite(distance) and range_min <= distance <= range_max:
                    distances.append(distance)

        return min(distances) if distances else float('inf')

    def get_warning(self, distance):

        if not math.isfinite(distance):
            return "NO OBSTACLE"
        elif distance < 1.5:
            return "DANGER"
        elif distance <= 3.0:
            return "CAUTION"
        else:
            return "SAFE"

    def lidar_callback(self, msg):

        front = self.get_min_distance(
            msg.ranges, msg.angle_min, msg.angle_increment,
            -20, 20, msg.range_min, msg.range_max
        )

        left = self.get_min_distance(
            msg.ranges, msg.angle_min, msg.angle_increment,
            20, 90, msg.range_min, msg.range_max
        )

        right = self.get_min_distance(
            msg.ranges, msg.angle_min, msg.angle_increment,
            -90, -20, msg.range_min, msg.range_max
        )

        current_time = self.get_clock().now()

        if (current_time - self.last_log_time).nanoseconds < 1_000_000_000:
            return

        self.last_log_time = current_time

        front_status = self.get_warning(front)
        left_status = self.get_warning(left)
        right_status = self.get_warning(right)

        decision = obstacle_avoidance(front, left, right)

        self.get_logger().info(
            f'Front: {front:.2f} m [{front_status}] | '
            f'Left: {left:.2f} m [{left_status}] | '
            f'Right: {right:.2f} m [{right_status}]\n'
            f'DECISION: {decision}'
        )


def main(args=None):

    rclpy.init(args=args)

    node = LidarProcessor()

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
