#!/usr/bin/env python3

import math
from enum import Enum

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from std_msgs.msg import Bool, String, Float32


class AvoidanceState(Enum):
    FOLLOW_MISSION = 0
    SIDESTEP = 1
    FORWARD_BYPASS = 2


class AvoidanceManagerNode(Node):
    def __init__(self) -> None:
        super().__init__('avoidance_manager_node')

        # -------------------------------------------------
        # Declare parameters
        # -------------------------------------------------
        self.declare_parameter('avoidance_offset', 4.0)
        self.declare_parameter('bypass_forward_distance', 3.0)
        self.declare_parameter('goal_tolerance', 0.5)
        self.declare_parameter('timer_rate_hz', 20.0)

        # -------------------------------------------------
        # Read parameters
        # -------------------------------------------------
        self.avoidance_offset = float(
            self.get_parameter('avoidance_offset').value
        )
        self.bypass_forward_distance = float(
            self.get_parameter('bypass_forward_distance').value
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

        self.desired_yaw_sub = self.create_subscription(
            Float32,
            '/mission/desired_yaw',
            self.desired_yaw_callback,
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
        # Publishers
        # -------------------------------------------------
        self.target_position_pub = self.create_publisher(
            Point,
            '/mission/target_position',
            10
        )

        self.target_yaw_pub = self.create_publisher(
            Float32,
            '/mission/target_yaw',
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
        self.have_desired_yaw = False
        self.have_obstacle_status = False
        self.have_direction = False

        self.desired_yaw = 0.0
        self.obstacle_detected = False
        self.preferred_direction = 'NONE'

        self.frozen_avoidance_yaw = 0.0
        self.avoidance_direction = 'NONE'

        self.sidestep_target = None
        self.forward_bypass_target = None

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
            f'bypass_forward_distance={self.bypass_forward_distance:.2f}, '
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

    def desired_yaw_callback(self, msg: Float32) -> None:
        self.desired_yaw = float(msg.data)

        if not self.have_desired_yaw:
            self.have_desired_yaw = True
            self.get_logger().info(
                f'First desired yaw received: {self.desired_yaw:.2f} rad'
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
        if not self.have_current_position:
            return

        if not self.have_desired_target:
            return

        if not self.have_desired_yaw:
            return

        if self.state == AvoidanceState.FOLLOW_MISSION:
            self.handle_follow_mission()

        elif self.state == AvoidanceState.SIDESTEP:
            self.handle_sidestep()

        elif self.state == AvoidanceState.FORWARD_BYPASS:
            self.handle_forward_bypass()

    # ------------------------------------------------------------------
    # State handlers
    # ------------------------------------------------------------------
    def handle_follow_mission(self) -> None:
        self.log_state_once(AvoidanceState.FOLLOW_MISSION)

        # Normal pass-through behavior
        self.target_position_pub.publish(self.desired_target)
        self.publish_target_yaw(self.desired_yaw)

        if not self.have_obstacle_status or not self.have_direction:
            return

        if not self.obstacle_detected:
            return

        if self.preferred_direction not in ['LEFT', 'RIGHT']:
            self.get_logger().warn(
                f'Obstacle detected but preferred_direction="{self.preferred_direction}" is invalid.'
            )
            return

        forward_x, forward_y = self.compute_forward_unit_vector()

        if forward_x is None or forward_y is None:
            self.get_logger().warn(
                'Could not compute forward vector. Ignoring avoidance request.'
            )
            return

        self.avoidance_direction = self.preferred_direction
        self.frozen_avoidance_yaw = self.desired_yaw

        self.sidestep_target = self.compute_sidestep_target(
            direction=self.avoidance_direction,
            forward_x=forward_x,
            forward_y=forward_y
        )

        self.forward_bypass_target = None
        self.state = AvoidanceState.SIDESTEP

        self.get_logger().warn(
            f'Obstacle detected. Entering SIDESTEP with direction={self.avoidance_direction}. '
            f'Sidestep target=({self.sidestep_target.x:.2f}, '
            f'{self.sidestep_target.y:.2f}, {self.sidestep_target.z:.2f}), '
            f'frozen_yaw={self.frozen_avoidance_yaw:.2f}'
        )

    def handle_sidestep(self) -> None:
        self.log_state_once(AvoidanceState.SIDESTEP)

        if self.sidestep_target is None:
            self.get_logger().warn(
                'SIDESTEP state without sidestep target. Returning to FOLLOW_MISSION.'
            )
            self.reset_avoidance()
            return

        self.target_position_pub.publish(self.sidestep_target)
        self.publish_target_yaw(self.frozen_avoidance_yaw)

        if self.is_goal_reached(self.sidestep_target):
            forward_x, forward_y = self.compute_forward_unit_vector()

            if forward_x is None or forward_y is None:
                self.get_logger().warn(
                    'Could not compute forward vector after sidestep. Returning to FOLLOW_MISSION.'
                )
                self.reset_avoidance()
                return

            self.forward_bypass_target = self.compute_forward_bypass_target(
                start_point=self.sidestep_target,
                forward_x=forward_x,
                forward_y=forward_y
            )

            self.state = AvoidanceState.FORWARD_BYPASS

            self.get_logger().info(
                f'Sidestep completed. Entering FORWARD_BYPASS with target='
                f'({self.forward_bypass_target.x:.2f}, '
                f'{self.forward_bypass_target.y:.2f}, '
                f'{self.forward_bypass_target.z:.2f})'
            )

    def handle_forward_bypass(self) -> None:
        self.log_state_once(AvoidanceState.FORWARD_BYPASS)

        if self.forward_bypass_target is None:
            self.get_logger().warn(
                'FORWARD_BYPASS state without forward target. Returning to FOLLOW_MISSION.'
            )
            self.reset_avoidance()
            return

        self.target_position_pub.publish(self.forward_bypass_target)
        self.publish_target_yaw(self.frozen_avoidance_yaw)

        if self.is_goal_reached(self.forward_bypass_target):
            self.get_logger().info(
                'Forward bypass completed. Returning to FOLLOW_MISSION.'
            )
            self.reset_avoidance()

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def publish_target_yaw(self, yaw: float) -> None:
        msg = Float32()
        msg.data = float(yaw)
        self.target_yaw_pub.publish(msg)

    def compute_forward_unit_vector(self):
        dx = self.desired_target.x - self.current_position.x
        dy = self.desired_target.y - self.current_position.y
        norm = math.sqrt(dx * dx + dy * dy)

        if norm < 1e-6:
            return None, None

        return dx / norm, dy / norm

    def compute_sidestep_target(
        self,
        direction: str,
        forward_x: float,
        forward_y: float
    ) -> Point:
        target = Point()
        target.z = self.desired_target.z

        if direction == 'LEFT':
            lateral_x = -forward_y
            lateral_y = forward_x
        else:
            lateral_x = forward_y
            lateral_y = -forward_x

        target.x = self.current_position.x + self.avoidance_offset * lateral_x
        target.y = self.current_position.y + self.avoidance_offset * lateral_y

        return target

    def compute_forward_bypass_target(
        self,
        start_point: Point,
        forward_x: float,
        forward_y: float
    ) -> Point:
        target = Point()
        target.x = start_point.x + self.bypass_forward_distance * forward_x
        target.y = start_point.y + self.bypass_forward_distance * forward_y
        target.z = self.desired_target.z
        return target

    def is_goal_reached(self, goal: Point) -> bool:
        dx = goal.x - self.current_position.x
        dy = goal.y - self.current_position.y
        dz = goal.z - self.current_position.z
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)
        return distance < self.goal_tolerance

    def reset_avoidance(self) -> None:
        self.sidestep_target = None
        self.forward_bypass_target = None
        self.avoidance_direction = 'NONE'
        self.state = AvoidanceState.FOLLOW_MISSION

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