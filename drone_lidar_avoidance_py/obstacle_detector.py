#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, Bool, String


class ObstacleDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__('obstacle_detector_node')

        # -------------------------------------------------
        # Parameters
        # -------------------------------------------------
        self.declare_parameter('obstacle_distance_threshold', 3.0)
        self.declare_parameter('log_rate_hz', 20.0)

        self.threshold = float(
            self.get_parameter('obstacle_distance_threshold').value
        )
        self.log_rate_hz = float(
            self.get_parameter('log_rate_hz').value
        )

        # -------------------------------------------------
        # Subscribers
        # -------------------------------------------------
        self.front_sub = self.create_subscription(
            Float32,
            '/avoidance/front_distance',
            self.front_callback,
            10
        )

        self.left_sub = self.create_subscription(
            Float32,
            '/avoidance/left_distance',
            self.left_callback,
            10
        )

        self.right_sub = self.create_subscription(
            Float32,
            '/avoidance/right_distance',
            self.right_callback,
            10
        )

        # -------------------------------------------------
        # Publishers
        # -------------------------------------------------
        self.obstacle_pub = self.create_publisher(
            Bool,
            '/avoidance/obstacle_detected',
            10
        )

        self.direction_pub = self.create_publisher(
            String,
            '/avoidance/preferred_direction',
            10
        )

        self.front_blocked_pub = self.create_publisher(
            Bool,
            '/avoidance/front_blocked',
            10
        )

        # -------------------------------------------------
        # State
        # -------------------------------------------------
        self.front = math.inf
        self.left = math.inf
        self.right = math.inf

        self.timer = self.create_timer(
            1.0 / self.log_rate_hz,
            self.timer_callback
        )

        self.get_logger().info('Obstacle detector node started')

    # -----------------------------------------------------
    # Callbacks
    # -----------------------------------------------------
    def front_callback(self, msg: Float32):
        self.front = msg.data

    def left_callback(self, msg: Float32):
        self.left = msg.data

    def right_callback(self, msg: Float32):
        self.right = msg.data

    def timer_callback(self):
        front_blocked = self.is_blocked(self.front)
        obstacle_detected = front_blocked

        direction = "NONE"

        if obstacle_detected:
            if self.left > self.right:
                direction = "LEFT"
            else:
                direction = "RIGHT"

        # Publish
        self.publish_bool(self.obstacle_pub, obstacle_detected)
        self.publish_bool(self.front_blocked_pub, front_blocked)
        self.publish_string(self.direction_pub, direction)

        # Log
        self.get_logger().info(
            f'Obstacle: {obstacle_detected} | '
            f'Front blocked: {front_blocked} | '
            f'Direction: {direction}'
        )

    # -----------------------------------------------------
    # Helpers
    # -----------------------------------------------------
    def is_blocked(self, distance: float) -> bool:
        if math.isinf(distance):
            return False
        return distance < self.threshold

    def publish_bool(self, pub, value: bool):
        msg = Bool()
        msg.data = value
        pub.publish(msg)

    def publish_string(self, pub, value: str):
        msg = String()
        msg.data = value
        pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down obstacle detector')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()