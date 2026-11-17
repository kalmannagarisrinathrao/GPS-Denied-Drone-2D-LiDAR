import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from px4_msgs.msg import ObstacleDistance


class PX4ObstacleBridge(Node):

    def __init__(self):
        super().__init__('px4_obstacle_bridge')

        self.subscription = self.create_subscription(
            LaserScan,
            '/world/walls/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan',
            self.lidar_callback,
            10
        )

        self.publisher = self.create_publisher(
            ObstacleDistance,
            '/fmu/in/obstacle_distance',
            10
        )

        self.get_logger().info('PX4 Obstacle Distance Bridge Started')

    def lidar_callback(self, scan):

        msg = ObstacleDistance()

        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.frame = ObstacleDistance.MAV_FRAME_BODY_FRD
        msg.sensor_type = ObstacleDistance.MAV_DISTANCE_SENSOR_LASER

        msg.increment = 5.0
        msg.angle_offset = 0.0
        msg.min_distance = 10
        msg.max_distance = 3000

        # 65535 = unknown; 3001 = clear beyond maximum range
        distances = [65535] * 72

        for i, distance in enumerate(scan.ranges):

            ros_angle = scan.angle_min + i * scan.angle_increment
            px4_angle = -math.degrees(ros_angle)

            if not -135.0 <= px4_angle <= 135.0:
                continue

            index = int(round(px4_angle / 5.0)) % 72

            # NaN and negative infinity are invalid/unknown
            if math.isnan(distance) or distance == -math.inf:
                continue

            # Positive infinity or beyond 30 m means clear
            if distance == math.inf or distance > 30.0:
                if distances[index] == 65535:
                    distances[index] = 3001
                continue

            if distance < scan.range_min:
                continue

            distance_cm = int(distance * 100)

            if distances[index] == 65535 or distances[index] == 3001:
                distances[index] = distance_cm
            else:
                distances[index] = min(
                    distances[index],
                    distance_cm
                )

        msg.distances = distances
        self.publisher.publish(msg)


def main(args=None):

    rclpy.init(args=args)
    node = PX4ObstacleBridge()

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
