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
    current_gripper: float = 0.0
    latest_target: np.ndarray | None = None
    latest_gripper: float = 0.0
    command: np.ndarray | None = None
    command_gripper: float = 0.0
    active: bool = False
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
        self.declare_parameter('max_joint_velocity_rad_s', 0.5)
        self.declare_parameter('max_gripper_velocity_m_s', 0.02)
        self.declare_parameter('command_rate_hz', 50.0)
        self.declare_parameter('target_timeout_s', 0.25)
        self.declare_parameter('log_joint_changes', False)
        self.declare_parameter('joint_log_rate_hz', 1.0)
        self.declare_parameter('joint_log_min_change_rad', 0.005)

        self._max_joint_velocity = max(
            0.01, float(self.get_parameter('max_joint_velocity_rad_s').value))
        self._max_gripper_velocity = max(
            0.001, float(self.get_parameter('max_gripper_velocity_m_s').value))
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
        self._last_tick_time = time.monotonic()
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f'KER arm bridge started, max_joint_velocity='
            f'{self._max_joint_velocity:.3f} rad/s')

    def _state_callback(self, message: JointState) -> None:
        by_name = dict(zip(message.name, message.position))
        for side, names in (('left', LEFT_TARGET_NAMES), ('right', RIGHT_TARGET_NAMES)):
            arm_names = names[:7]
            if all(name in by_name for name in arm_names):
                arm = self._arms[side]
                arm.current = np.array([by_name[name] for name in arm_names])
                if names[7] in by_name:
                    arm.current_gripper = float(by_name[names[7]])

    def _target_callback(self, side: str, message: JointState) -> None:
        names = LEFT_TARGET_NAMES if side == 'left' else RIGHT_TARGET_NAMES
        by_name = dict(zip(message.name, message.position))
        if not all(name in by_name for name in names):
            self.get_logger().warn(f'Ignoring incomplete {side} KER target')
            return
        arm = self._arms[side]
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
        arm.command = arm.current.copy()
        arm.command_gripper = arm.current_gripper
        arm.active = True
        self.get_logger().info(f'[{side}] rate-limited takeover started')

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
        elapsed = min(0.1, max(0.0, now - self._last_tick_time))
        self._last_tick_time = now
        joint_step = self._max_joint_velocity * elapsed
        gripper_step = self._max_gripper_velocity * elapsed
        for side, arm in self._arms.items():
            if not arm.active or arm.latest_target is None or arm.command is None:
                continue
            if now - self._last_target_time[side] > self._timeout:
                continue
            arm.command += np.clip(
                arm.latest_target - arm.command, -joint_step, joint_step)
            gripper_delta = arm.latest_gripper - arm.command_gripper
            arm.command_gripper += float(np.clip(
                gripper_delta, -gripper_step, gripper_step))
            self._publish(side, arm.command, arm.command_gripper)

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
