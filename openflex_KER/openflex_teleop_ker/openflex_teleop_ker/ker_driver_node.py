#!/usr/bin/env python3
"""Read a KER device and publish source and target ROS 2 JointState messages."""

import json
import threading
import time

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String, UInt16
from std_srvs.srv import Trigger

from .ker_stream import CMD_STREAM, KERStream
from .pose_processor import (
    LEFT_TARGET_NAMES,
    RIGHT_TARGET_NAMES,
    SOURCE_JOINT_NAMES,
    KerPoseProcessor,
)


class KerDriverNode(Node):
    def __init__(self):
        super().__init__('openflex_ker_driver')
        self.declare_parameter('transport', 'usb')
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 2000000)
        self.declare_parameter('usb_vid', 0x303A)
        self.declare_parameter('usb_pid', 0x4002)
        self.declare_parameter('wifi_host', 'openarm-ker.local')
        self.declare_parameter('wifi_port', 19090)
        self.declare_parameter('wifi_connect_timeout_s', 3.0)
        self.declare_parameter('wifi_socket_timeout_s', 0.02)
        self.declare_parameter('publish_rate_hz', 100.0)
        self.declare_parameter('reconnect_interval_s', 2.0)
        self.declare_parameter('use_hampel_filter', False)
        self.declare_parameter('use_low_pass_filter', True)
        self.declare_parameter('low_pass_alpha', 0.2)
        self.declare_parameter('drop_command_on_sensor_error', True)
        self.declare_parameter('gripper_min_position', 0.0)
        self.declare_parameter('gripper_max_position', 0.044)
        self.declare_parameter(
            'joint_scales',
            [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, -1.0,
             1.0, 1.0, 1.0, 1.0, 1.0, 1.0, -1.0],
        )
        self.declare_parameter('joint_offsets', [0.0] * 14)
        self.declare_parameter('source_joint_topic', '/ker/joint_states')
        self.declare_parameter('error_mask_topic', '/ker/error_mask')
        self.declare_parameter('left_target_topic', '/ker/left_arm/joint_command')
        self.declare_parameter('right_target_topic', '/ker/right_arm/joint_command')
        self.declare_parameter('ping_service', '/ker/ping')
        self.declare_parameter('ping_usb_service', '/ker/ping_usb')
        self.declare_parameter('ping_wifi_service', '/ker/ping_wifi')

        self._use_hampel = bool(self.get_parameter('use_hampel_filter').value)
        self._use_low_pass = bool(self.get_parameter('use_low_pass_filter').value)
        self._low_pass_alpha = float(self.get_parameter('low_pass_alpha').value)
        self._processor = KerPoseProcessor(
            use_hampel=self._use_hampel,
            use_low_pass=self._use_low_pass,
            low_pass_alpha=self._low_pass_alpha,
            gripper_min=float(self.get_parameter('gripper_min_position').value),
            gripper_max=float(self.get_parameter('gripper_max_position').value),
            joint_scales=self.get_parameter('joint_scales').value,
            joint_offsets=self.get_parameter('joint_offsets').value,
        )
        self._drop_on_error = bool(self.get_parameter('drop_command_on_sensor_error').value)
        self._source_pub = self.create_publisher(
            JointState, self.get_parameter('source_joint_topic').value, 10)
        self._error_pub = self.create_publisher(
            UInt16, self.get_parameter('error_mask_topic').value, 10)
        self._left_pub = self.create_publisher(
            JointState, self.get_parameter('left_target_topic').value, 10)
        self._right_pub = self.create_publisher(
            JointState, self.get_parameter('right_target_topic').value, 10)
        self._metadata_pub = self.create_publisher(String, '/ker/metadata', 1)
        self._ping_service = self.create_service(
            Trigger, self.get_parameter('ping_service').value, self._ping_callback)
        self._ping_usb_service = self.create_service(
            Trigger, self.get_parameter('ping_usb_service').value,
            lambda request, response: self._ping_callback_for(
                'usb', request, response))
        self._ping_wifi_service = self.create_service(
            Trigger, self.get_parameter('ping_wifi_service').value,
            lambda request, response: self._ping_callback_for(
                'wifi', request, response))

        self._stream = None
        self._stream_lock = threading.RLock()
        self._last_connect_attempt_ns = 0
        self.add_on_set_parameters_callback(self._on_set_parameters)
        rate = max(1.0, float(self.get_parameter('publish_rate_hz').value))
        self._timer = self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info('KER driver started; waiting for device')

    def _on_set_parameters(self, parameters):
        use_hampel = self._use_hampel
        use_low_pass = self._use_low_pass
        low_pass_alpha = self._low_pass_alpha
        filters_changed = False
        for parameter in parameters:
            if parameter.name == 'use_hampel_filter':
                use_hampel = bool(parameter.value)
                filters_changed = True
            elif parameter.name == 'use_low_pass_filter':
                use_low_pass = bool(parameter.value)
                filters_changed = True
            elif parameter.name == 'low_pass_alpha':
                low_pass_alpha = float(parameter.value)
                filters_changed = True
        if not 0.0 < low_pass_alpha <= 1.0:
            return SetParametersResult(
                successful=False,
                reason='low_pass_alpha must be in the range (0.0, 1.0]',
            )
        if filters_changed:
            self._processor.configure_filters(
                use_hampel=use_hampel,
                use_low_pass=use_low_pass,
                low_pass_alpha=low_pass_alpha,
            )
            self._use_hampel = use_hampel
            self._use_low_pass = use_low_pass
            self._low_pass_alpha = low_pass_alpha
        return SetParametersResult(successful=True)

    def _connect(self, force: bool = False) -> bool:
        reconnect_ns = int(float(self.get_parameter('reconnect_interval_s').value) * 1e9)
        now_ns = self.get_clock().now().nanoseconds
        if not force and now_ns - self._last_connect_attempt_ns < reconnect_ns:
            return False
        self._last_connect_attempt_ns = now_ns
        with self._stream_lock:
            self._close_stream()
            try:
                self._stream = self._make_stream(
                    str(self.get_parameter('transport').value))
                self._stream.connect()
                self._stream.send_command(CMD_STREAM)
                self._publish_metadata(self._stream)
                self.get_logger().info(
                    f'KER connected via {self._stream.transport}: '
                    f'{self._stream.metadata}')
                return True
            except Exception as error:
                self.get_logger().warn(f'KER connection failed: {error}')
                self._close_stream()
                return False

    def _make_stream(self, transport: str) -> KERStream:
        return KERStream(
            transport=transport,
            port=str(self.get_parameter('serial_port').value),
            baud=int(self.get_parameter('baud_rate').value),
            vid=int(self.get_parameter('usb_vid').value),
            pid=int(self.get_parameter('usb_pid').value),
            wifi_host=str(self.get_parameter('wifi_host').value),
            wifi_port=int(self.get_parameter('wifi_port').value),
            connect_timeout=float(
                self.get_parameter('wifi_connect_timeout_s').value),
            socket_timeout=float(
                self.get_parameter('wifi_socket_timeout_s').value),
        )

    def _publish_metadata(self, stream: KERStream) -> None:
        metadata = dict(stream.metadata)
        metadata.update({
            'transport': stream.transport,
            'endpoint': stream.endpoint,
        })
        self._metadata_pub.publish(String(data=json.dumps(metadata)))

    def _ping_callback(self, _request, response):
        transport = str(self.get_parameter('transport').value)
        return self._ping_callback_for(transport, _request, response)

    def _ping_callback_for(self, transport, _request, response):
        """Perform a real PING/schema handshake on the selected transport."""
        start = time.monotonic()
        active_transport = str(self.get_parameter('transport').value)
        stream = None
        try:
            with self._stream_lock:
                if transport == active_transport:
                    self._close_stream()
                stream = self._make_stream(transport)
                stream.connect()
                elapsed_ms = (time.monotonic() - start) * 1000.0
                metadata = stream.metadata
                response.success = True
                response.message = (
                    f'transport={transport} | endpoint={stream.endpoint} | '
                    f'latency={elapsed_ms:.1f} ms | '
                    f"HW={metadata.get('hw', 'unknown')} | "
                    f"FW={metadata.get('fw', 'unknown')} | "
                    f"updated={metadata.get('updated', 'unknown')}")
                if transport == active_transport:
                    stream.send_command(CMD_STREAM)
                    self._stream = stream
                    stream = None
                    self._publish_metadata(self._stream)
        except Exception as error:
            elapsed_ms = (time.monotonic() - start) * 1000.0
            endpoint = stream.endpoint if stream is not None else self._make_stream(transport).endpoint
            response.success = False
            response.message = (
                f'transport={transport} | endpoint={endpoint} | '
                f'latency={elapsed_ms:.1f} ms | error={error}')
            self.get_logger().warn(f'KER {transport} Ping failed: {error}')
        finally:
            if stream is not None:
                stream.close()
        return response

    def _tick(self) -> None:
        with self._stream_lock:
            if self._stream is None or not self._stream.is_connected:
                self._connect()
                return
            data = self._stream.recv()
        if data is None:
            return
        angles = data.get('angles')
        if not isinstance(angles, list) or len(angles) != 16:
            self.get_logger().error('KER packet does not contain 16 angles')
            return
        errors = list(data.get('errors', [False] * 16))
        error_mask = sum((1 << index) for index, error in enumerate(errors) if error)
        self._error_pub.publish(UInt16(data=error_mask))

        pose = self._processor.process(angles)
        stamp = self.get_clock().now().to_msg()
        self._publish_joint_state(self._source_pub, SOURCE_JOINT_NAMES, pose.source_radians, stamp)
        if not (self._drop_on_error and error_mask):
            self._publish_joint_state(self._right_pub, RIGHT_TARGET_NAMES, pose.right_target, stamp)
            self._publish_joint_state(self._left_pub, LEFT_TARGET_NAMES, pose.left_target, stamp)

    @staticmethod
    def _publish_joint_state(publisher, names, positions, stamp) -> None:
        message = JointState()
        message.header.stamp = stamp
        message.name = list(names)
        message.position = list(positions)
        publisher.publish(message)

    def _close_stream(self) -> None:
        if self._stream is not None:
            self._stream.close()
            self._stream = None

    def destroy_node(self):
        self._close_stream()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KerDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
