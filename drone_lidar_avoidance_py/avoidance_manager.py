#!/usr/bin/env python3

import math
from enum import Enum

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from std_msgs.msg import Bool, String


class AvoidanceState(Enum):
    FOLLOW_MISSION = 0
    AVOIDING = 1


class AvoidanceManagerNode(Node):
    def __init__(self) -> None:
        super().__init__('avoidance_manager_node')

        # -------------------------------------------------
        # Declare parameters
        # -------------------------------------------------
        self.declare_parameter('avoidance_offset', 2.0)
        self.declare_parameter('goal_tolerance', 0.5)
        self.declare_parameter('timer_rate_hz', 10.0)

        # -------------------------------------------------
        # Read parameters
        # -------------------------------------------------
        self.avoidance_offset = float(
            self.get_parameter('avoidance_offset').value
        )
        self.goal_tolerance = float(
            self.get_parameter('goal_tolerance').value
        )
        self.timer_rate_hz = float(
            self.get_parameter('timer_rate_hz').value
        )

        if self.timer_rate_hz <= 0.0:
            raise ValueError('Parameter "timer_rate_hz" must be > 0.0')

        # -------------------------------------------------
        # Subscribers
        # -------------------------------------------------
        self.desired_target_sub = self.create_subscription(
            Point,
            '/mission/desired_target',
            self.desired_target_callback,
            10
        )

        self.current_position_sub = self.create_subscription(
            Point,
            '/mission/current_position',
            self.current_position_callback,
            10
        )

        self.obstacle_detected_sub = self.create_subscription(
            Bool,
            '/avoidance/obstacle_detected',
            self.obstacle_detected_callback,
            10
        )

        self.preferred_direction_sub = self.create_subscription(
            String,
            '/avoidance/preferred_direction',
            self.preferred_direction_callback,
            10
        )

        # -------------------------------------------------
        # Publisher
        # -------------------------------------------------
        self.target_position_pub = self.create_publisher(
            Point,
            '/mission/target_position',
            10
        )

        # -------------------------------------------------
        # Internal state
        # -------------------------------------------------
        self.state = AvoidanceState.FOLLOW_MISSION

        self.current_position = Point()
        self.desired_target = Point()

        self.have_current_position = False
        self.have_desired_target = False
        self.have_obstacle_status = False
        self.have_direction = False

        self.obstacle_detected = False
        self.preferred_direction = 'NONE'

        self.avoidance_target = None
        self.last_logged_state = None

        self.timer = self.create_timer(
            1.0 / self.timer_rate_hz,
            self.timer_callback
        )

        # -------------------------------------------------
        # Startup logs
        # -------------------------------------------------
        self.get_logger().info('Avoidance manager node started')
        self.get_logger().info(
            f'Parameters: avoidance_offset={self.avoidance_offset:.2f}, '
            f'goal_tolerance={self.goal_tolerance:.2f}, '
            f'timer_rate_hz={self.timer_rate_hz:.2f}'
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def desired_target_callback(self, msg: Point) -> None:
        self.desired_target = msg

        if not self.have_desired_target:
            self.have_desired_target = True
            self.get_logger().info(
                f'First desired mission target received: '
                f'({msg.x:.2f}, {msg.y:.2f}, {msg.z:.2f})'
            )

    def current_position_callback(self, msg: Point) -> None:
        self.current_position = msg

        if not self.have_current_position:
            self.have_current_position = True
            self.get_logger().info(
                f'First current position received: '
                f'({msg.x:.2f}, {msg.y:.2f}, {msg.z:.2f})'
            )

    def obstacle_detected_callback(self, msg: Bool) -> None:
        self.obstacle_detected = msg.data
        self.have_obstacle_status = True

    def preferred_direction_callback(self, msg: String) -> None:
        self.preferred_direction = msg.data
        self.have_direction = True

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------
    def timer_callback(self) -> None:
        if not self.have_current_position or not self.have_desired_target:
            self.get_logger().debug('Waiting for current position and desired target...')
            return

        if self.state == AvoidanceState.FOLLOW_MISSION:
            self.handle_follow_mission()

        elif self.state == AvoidanceState.AVOIDING:
            self.handle_avoiding()

    # ------------------------------------------------------------------
    # State handlers
    # ------------------------------------------------------------------
    def handle_follow_mission(self) -> None:
        self.log_state_once(AvoidanceState.FOLLOW_MISSION)

        # Default behavior: pass mission target straight through
        self.target_position_pub.publish(self.desired_target)

        # Only consider avoidance after detector outputs are available
        if not self.have_obstacle_status or not self.have_direction:
            return

        if self.obstacle_detected:
            direction = self.preferred_direction

            if direction not in ['LEFT', 'RIGHT']:
                self.get_logger().warn(
                    f'Obstacle detected but preferred_direction="{direction}" is invalid. '
                    'Ignoring avoidance request.'
                )
                return

            self.avoidance_target = self.compute_avoidance_target(direction)
            self.state = AvoidanceState.AVOIDING

            self.get_logger().warn(
                f'Obstacle detected. Switching to AVOIDING with direction={direction}. '
                f'Avoidance target=({self.avoidance_target.x:.2f}, '
                f'{self.avoidance_target.y:.2f}, {self.avoidance_target.z:.2f})'
            )

    def handle_avoiding(self) -> None:
        self.log_state_once(AvoidanceState.AVOIDING)

        if self.avoidance_target is None:
            self.get_logger().warn('AVOIDING state without avoidance target. Returning to FOLLOW_MISSION.')
            self.state = AvoidanceState.FOLLOW_MISSION
            return

        self.target_position_pub.publish(self.avoidance_target)

        if self.is_goal_reached(self.avoidance_target):
            self.get_logger().info('Avoidance target reached. Returning to FOLLOW_MISSION.')
            self.avoidance_target = None
            self.state = AvoidanceState.FOLLOW_MISSION

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def compute_avoidance_target(self, direction: str) -> Point:
        target = Point()

        # Keep the same altitude
        target.z = self.desired_target.z

        # Start from current XY position
        target.x = self.current_position.x
        target.y = self.current_position.y

        # Simple first strategy:
        # sidestep in Y only, while holding current X and mission altitude
        if direction == 'LEFT':
            target.y += self.avoidance_offset
        elif direction == 'RIGHT':
            target.y -= self.avoidance_offset

        return target

    def is_goal_reached(self, goal: Point) -> bool:
        dx = goal.x - self.current_position.x
        dy = goal.y - self.current_position.y
        dz = goal.z - self.current_position.z
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)
        return distance < self.goal_tolerance

    def log_state_once(self, state: AvoidanceState) -> None:
        if self.last_logged_state != state:
            self.get_logger().info(f'Avoidance state: {state.name}')
            self.last_logged_state = state


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AvoidanceManagerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down avoidance manager node')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()