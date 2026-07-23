"""Pure KER angle conversion and filtering utilities."""

from collections import deque
from dataclasses import dataclass
import math
import statistics
from typing import Sequence


RIGHT_SOURCE_NAMES = [f'ker_right_joint{i}' for i in range(1, 8)] + ['ker_right_gripper']
LEFT_SOURCE_NAMES = [f'ker_left_joint{i}' for i in range(1, 8)] + ['ker_left_gripper']
SOURCE_JOINT_NAMES = RIGHT_SOURCE_NAMES + LEFT_SOURCE_NAMES

RIGHT_TARGET_NAMES = [f'openarmx_right_joint{i}' for i in range(1, 8)] + [
    'openarmx_right_finger_joint1'
]
LEFT_TARGET_NAMES = [f'openarmx_left_joint{i}' for i in range(1, 8)] + [
    'openarmx_left_finger_joint1'
]


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def map_range(value: float, in_min: float, in_max: float, out_min: float, out_max: float) -> float:
    """Map a clipped value between ranges, including descending input ranges."""
    low, high = sorted((in_min, in_max))
    value = clamp(value, low, high)
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


class HampelFilter:
    """Per-channel causal Hampel filter used to suppress encoder spikes."""

    def __init__(self, channels: int = 16, window_size: int = 5,
                 n_sigmas: float = 3.0, min_threshold_deg: float = 5.0):
        self._history = [deque(maxlen=window_size) for _ in range(channels)]
        self._n_sigmas = n_sigmas
        self._min_threshold = min_threshold_deg

    def process(self, values: Sequence[float]) -> list[float]:
        filtered = []
        for index, value in enumerate(values):
            history = self._history[index]
            output = float(value)
            if history:
                median = statistics.median(history)
                mad = statistics.median(abs(item - median) for item in history)
                threshold = max(self._n_sigmas * 1.4826 * mad, self._min_threshold)
                if abs(output - median) > threshold:
                    output = median
            history.append(output)
            filtered.append(output)
        return filtered


class LowPassFilter:
    """Per-channel exponential moving average filter."""

    def __init__(self, channels: int = 16, alpha: float = 0.2):
        if not 0.0 < alpha <= 1.0:
            raise ValueError('low_pass_alpha must be in the range (0.0, 1.0]')
        self._alpha = alpha
        self._state: list[float | None] = [None] * channels

    def process(self, values: Sequence[float]) -> list[float]:
        filtered = []
        for index, value in enumerate(values):
            current = float(value)
            previous = self._state[index]
            output = current if previous is None else (
                self._alpha * current + (1.0 - self._alpha) * previous
            )
            self._state[index] = output
            filtered.append(output)
        return filtered


@dataclass(frozen=True)
class KerPose:
    source_radians: list[float]
    right_target: list[float]
    left_target: list[float]


class KerPoseProcessor:
    """Convert firmware angles into source display values and OpenFlex commands."""

    def __init__(self, *, use_hampel: bool = False, use_low_pass: bool = False,
                 low_pass_alpha: float = 0.2, gripper_min: float = 0.0,
                 gripper_max: float = 0.044, joint_scales: Sequence[float] | None = None,
                 joint_offsets: Sequence[float] | None = None):
        self._hampel_filter = HampelFilter() if use_hampel else None
        self._low_pass_filter = LowPassFilter(alpha=low_pass_alpha) if use_low_pass else None
        self._gripper_min = gripper_min
        self._gripper_max = gripper_max
        self._scales = list(joint_scales or [1.0] * 14)
        self._offsets = list(joint_offsets or [0.0] * 14)
        if len(self._scales) != 14 or len(self._offsets) != 14:
            raise ValueError('joint_scales and joint_offsets must each contain 14 values')

    def configure_filters(self, *, use_hampel: bool, use_low_pass: bool,
                          low_pass_alpha: float) -> None:
        self._hampel_filter = HampelFilter() if use_hampel else None
        self._low_pass_filter = (
            LowPassFilter(alpha=low_pass_alpha) if use_low_pass else None
        )

    def process(self, raw_angles_deg: Sequence[float]) -> KerPose:
        if len(raw_angles_deg) != 16:
            raise ValueError(f'KER firmware must provide 16 angles, received {len(raw_angles_deg)}')
        angles = list(map(float, raw_angles_deg))
        if self._hampel_filter:
            angles = self._hampel_filter.process(angles)
        if self._low_pass_filter:
            angles = self._low_pass_filter.process(angles)

        source_radians = [math.radians(value) for value in angles]
        right_joints = self._transform_arm(angles[0:7], 0)
        left_joints = self._transform_arm(angles[8:15], 7)

        # KER angle 0 is open and its travel endpoint is closed. OpenFlex uses
        # joint 0 as closed and 0.044 as open, so the output range is reversed.
        right_gripper = map_range(angles[7], 0.0, -60.0,
                                  self._gripper_max, self._gripper_min)
        left_gripper = map_range(angles[15], 0.0, 60.0,
                                 self._gripper_max, self._gripper_min)
        return KerPose(
            source_radians=source_radians,
            right_target=right_joints + [right_gripper],
            left_target=left_joints + [left_gripper],
        )

    def _transform_arm(self, angles_deg: Sequence[float], offset: int) -> list[float]:
        return [
            math.radians(value) * self._scales[offset + index] + self._offsets[offset + index]
            for index, value in enumerate(angles_deg)
        ]
