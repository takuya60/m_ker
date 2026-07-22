#!/usr/bin/env python3
"""Safely bridge KER JointState targets to OpenFlex controllers."""

from dataclasses import dataclass
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from .pose_processor import LEFT_TARGET_NAMES, RIGHT_TARGET_NAMES


@dataclass
class ArmState:
    current: np.ndarray | None = None
    latest_target: np.ndarray | None = None
    latest_gripper: float = 0.0
    interpolation_start: np.ndarray | None = None
    interpolation_target: np.ndarray | None = None
    interpolation_started: float = 0.0
    active: bool = False
    rejected: bool = False
    warned_no_state: bool = False
    last_logged_target: np.ndarray | None = None
    last_log_time: float = 0.0


class KerArmBridgeNode(Node):
    def __init__(self):
        super().__init__('openflex_ker_arm_bridge')
        self.declare_parameter('left_target_topic', '/ker/left_arm/joint_command')
        self.declare_parameter('right_target_topic', '/ker/right_arm/joint_command')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter(
            'left_controller_topic', '/left_forward_position_controller/commands')
        self.declare_parameter(
            'right_controller_topic', '/right_forward_position_controller/commands')
        self.declare_parameter('enable_safety_check', True)
        self.declare_parameter('max_joint_diff_rad', 0.873)
        self.declare_parameter('interpolation_duration', 3.0)
        self.declare_parameter('command_rate_hz', 50.0)
        self.declare_parameter('target_timeout_s', 0.25)
        self.declare_parameter('log_joint_changes', False)
        self.declare_parameter('joint_log_rate_hz', 1.0)
        self.declare_parameter('joint_log_min_change_rad', 0.005)

        self._enable_safety = bool(self.get_parameter('enable_safety_check').value)
        self._max_diff = float(self.get_parameter('max_joint_diff_rad').value)
        self._duration = max(0.01, float(self.get_parameter('interpolation_duration').value))
        self._timeout = max(0.01, float(self.get_parameter('target_timeout_s').value))
        self._log_joint_changes = bool(
            self.get_parameter('log_joint_changes').value)
        log_rate = max(0.1, float(self.get_parameter('joint_log_rate_hz').value))
        self._joint_log_period = 1.0 / log_rate
        self._joint_log_min_change = max(
            0.0, float(self.get_parameter('joint_log_min_change_rad').value))
        self._last_target_time = {'left': 0.0, 'right': 0.0}
        self._arms = {'left': ArmState(), 'right': ArmState()}

        self.create_subscription(JointState, self.get_parameter('joint_states_topic').value,
                                 self._state_callback, 10)
        self.create_subscription(JointState, self.get_parameter('left_target_topic').value,
                                 lambda msg: self._target_callback('left', msg), 10)
        self.create_subscription(JointState, self.get_parameter('right_target_topic').value,
                                 lambda msg: self._target_callback('right', msg), 10)
        self._publishers = {
            'left': self.create_publisher(
                Float64MultiArray, self.get_parameter('left_controller_topic').value, 10),
            'right': self.create_publisher(
                Float64MultiArray, self.get_parameter('right_controller_topic').value, 10),
        }
        rate = max(1.0, float(self.get_parameter('command_rate_hz').value))
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f'KER arm bridge started, safety={self._enable_safety}, '
            f'max_diff={self._max_diff:.3f} rad')

    def _state_callback(self, message: JointState) -> None:
        by_name = dict(zip(message.name, message.position))
        for side, names in (('left', LEFT_TARGET_NAMES), ('right', RIGHT_TARGET_NAMES)):
            arm_names = names[:7]
            if all(name in by_name for name in arm_names):
                self._arms[side].current = np.array([by_name[name] for name in arm_names])

    def _target_callback(self, side: str, message: JointState) -> None:
        names = LEFT_TARGET_NAMES if side == 'left' else RIGHT_TARGET_NAMES
        by_name = dict(zip(message.name, message.position))
        if not all(name in by_name for name in names):
            self.get_logger().warn(f'Ignoring incomplete {side} KER target')
            return
        arm = self._arms[side]
        if arm.rejected:
            return
        target = np.array([by_name[name] for name in names[:7]], dtype=float)
        arm.latest_target = target
        arm.latest_gripper = float(by_name[names[7]])
        self._last_target_time[side] = time.monotonic()
        self._log_target_if_changed(side, arm, target)

        if arm.active:
            return
        if arm.current is None:
            if not arm.warned_no_state:
                self.get_logger().warn(f'[{side}] waiting for robot /joint_states')
                arm.warned_no_state = True
            return
        differences = np.abs(target - arm.current)
        max_difference = float(np.max(differences))
        if self._enable_safety and max_difference > self._max_diff:
            joint = int(np.argmax(differences)) + 1
            arm.rejected = True
            self.get_logger().error(
                f'[{side}] takeover rejected: joint{joint} differs by {max_difference:.3f} rad; '
                f'limit is {self._max_diff:.3f} rad. Match KER and robot poses, then restart '
                'this node.')
            return
        arm.interpolation_start = arm.current.copy()
        arm.interpolation_target = target.copy()
        arm.interpolation_started = time.monotonic()
        arm.active = True
        self.get_logger().info(f'[{side}] safe takeover started')

    def _log_target_if_changed(
            self, side: str, arm: ArmState, target: np.ndarray) -> None:
        if not self._log_joint_changes:
            return
        now = time.monotonic()
        if now - arm.last_log_time < self._joint_log_period:
            return
        if arm.last_logged_target is not None:
            max_change = float(np.max(np.abs(target - arm.last_logged_target)))
            if max_change < self._joint_log_min_change:
                return
        values = ', '.join(
            f'J{index}={value:+.3f}' for index, value in enumerate(target, start=1))
        self.get_logger().info(f'[{side}] KER target [rad]: {values}')
        arm.last_logged_target = target.copy()
        arm.last_log_time = now

    def _tick(self) -> None:
        now = time.monotonic()
        for side, arm in self._arms.items():
            if not arm.active or arm.latest_target is None:
                continue
            if now - self._last_target_time[side] > self._timeout:
                continue
            elapsed = now - arm.interpolation_started
            if elapsed < self._duration:
                fraction = elapsed / self._duration
                command = arm.interpolation_start + (
                    arm.interpolation_target - arm.interpolation_start) * fraction
            else:
                command = arm.latest_target
            self._publish(side, command, arm.latest_gripper)

    def _publish(self, side: str, positions: np.ndarray, gripper: float) -> None:
        message = Float64MultiArray()
        message.data = positions.tolist() + [gripper]
        self._publishers[side].publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = KerArmBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
