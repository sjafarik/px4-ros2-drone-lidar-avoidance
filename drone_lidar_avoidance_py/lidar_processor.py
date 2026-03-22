#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32


class LidarProcessorNode(Node):
    def __init__(self) -> None:
        super().__init__('lidar_processor_node')

        # -------------------------------------------------
        # Declare parameters
        # -------------------------------------------------
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('front_angle_deg', 20.0)
        self.declare_parameter('side_min_angle_deg', 20.0)
        self.declare_parameter('side_max_angle_deg', 100.0)
        self.declare_parameter('log_rate_hz', 1.0)

        # -------------------------------------------------
        # Read parameters
        # -------------------------------------------------
        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.front_angle_deg = float(self.get_parameter('front_angle_deg').value)
        self.side_min_angle_deg = float(self.get_parameter('side_min_angle_deg').value)
        self.side_max_angle_deg = float(self.get_parameter('side_max_angle_deg').value)
        self.log_rate_hz = float(self.get_parameter('log_rate_hz').value)

        if self.log_rate_hz <= 0.0:
            raise ValueError('Parameter "log_rate_hz" must be > 0.0')

        if self.front_angle_deg <= 0.0:
            raise ValueError('Parameter "front_angle_deg" must be > 0.0')

        if self.side_min_angle_deg < 0.0 or self.side_max_angle_deg <= self.side_min_angle_deg:
            raise ValueError(
                'Parameters must satisfy: 0.0 <= side_min_angle_deg < side_max_angle_deg'
            )

        # -------------------------------------------------
        # Publishers / Subscribers
        # -------------------------------------------------
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            10
        )

        self.front_distance_pub = self.create_publisher(
            Float32,
            '/avoidance/front_distance',
            10
        )

        self.left_distance_pub = self.create_publisher(
            Float32,
            '/avoidance/left_distance',
            10
        )

        self.right_distance_pub = self.create_publisher(
            Float32,
            '/avoidance/right_distance',
            10
        )

        # -------------------------------------------------
        # Internal state
        # -------------------------------------------------
        self.have_scan = False

        self.front_min_distance = math.inf
        self.left_min_distance = math.inf
        self.right_min_distance = math.inf

        self.log_timer = self.create_timer(1.0 / self.log_rate_hz, self.log_timer_callback)

        # -------------------------------------------------
        # Startup logs
        # -------------------------------------------------
        self.get_logger().info('LiDAR processor node started')
        self.get_logger().info(
            f'Parameters: scan_topic={self.scan_topic}, '
            f'front_angle_deg={self.front_angle_deg:.1f}, '
            f'side_min_angle_deg={self.side_min_angle_deg:.1f}, '
            f'side_max_angle_deg={self.side_max_angle_deg:.1f}, '
            f'log_rate_hz={self.log_rate_hz:.1f}'
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def scan_callback(self, msg: LaserScan) -> None:
        self.have_scan = True

        self.front_min_distance = self.compute_sector_min_distance(
            msg=msg,
            angle_min_deg=-self.front_angle_deg,
            angle_max_deg=self.front_angle_deg
        )

        self.left_min_distance = self.compute_sector_min_distance(
            msg=msg,
            angle_min_deg=self.side_min_angle_deg,
            angle_max_deg=self.side_max_angle_deg
        )

        self.right_min_distance = self.compute_sector_min_distance(
            msg=msg,
            angle_min_deg=-self.side_max_angle_deg,
            angle_max_deg=-self.side_min_angle_deg
        )

        self.publish_distance('/avoidance/front_distance', self.front_distance_pub, self.front_min_distance)
        self.publish_distance('/avoidance/left_distance', self.left_distance_pub, self.left_min_distance)
        self.publish_distance('/avoidance/right_distance', self.right_distance_pub, self.right_min_distance)

    def log_timer_callback(self) -> None:
        if not self.have_scan:
            self.get_logger().info('Waiting for LiDAR scan...')
            return

        self.get_logger().info(
            f'Sector distances | '
            f'front={self.format_distance(self.front_min_distance)}, '
            f'left={self.format_distance(self.left_min_distance)}, '
            f'right={self.format_distance(self.right_min_distance)}'
        )

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def compute_sector_min_distance(
        self,
        msg: LaserScan,
        angle_min_deg: float,
        angle_max_deg: float
    ) -> float:
        valid_ranges = []

        for i, distance in enumerate(msg.ranges):
            angle_rad = msg.angle_min + i * msg.angle_increment
            angle_deg = math.degrees(angle_rad)

            if angle_deg < angle_min_deg or angle_deg > angle_max_deg:
                continue

            if self.is_valid_range(distance, msg.range_min, msg.range_max):
                valid_ranges.append(distance)

        if not valid_ranges:
            return math.inf

        return min(valid_ranges)

    def is_valid_range(self, distance: float, range_min: float, range_max: float) -> bool:
        if math.isinf(distance) or math.isnan(distance):
            return False

        return range_min <= distance <= range_max

    def publish_distance(self, topic_name: str, publisher, distance: float) -> None:
        msg = Float32()

        if math.isinf(distance):
            msg.data = float('inf')
        else:
            msg.data = float(distance)

        publisher.publish(msg)

    def format_distance(self, distance: float) -> str:
        if math.isinf(distance):
            return 'inf'
        return f'{distance:.2f} m'


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarProcessorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down LiDAR processor node')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()