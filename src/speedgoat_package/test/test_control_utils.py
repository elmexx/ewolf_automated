import math

from speedgoat_package.control_utils import (
    DelayedPathBuffer,
    SteeringProtectionConfig,
    SteeringProtector,
    mean_abs_path_delta,
)


def test_stable_input_remains_stable():
    protector = SteeringProtector(
        SteeringProtectionConfig(max_angle_rad=math.radians(30.0), smoothing_alpha=0.3)
    )

    outputs = [protector.apply(math.radians(5.0), 0.1).final_angle_rad for _ in range(5)]
    assert all(abs(v - outputs[-1]) < math.radians(5.0) for v in outputs[1:])
    assert outputs[-1] > 0.0


def test_single_frame_jump_is_rate_limited():
    protector = SteeringProtector(
        SteeringProtectionConfig(
            max_angle_rad=math.radians(30.0),
            smoothing_alpha=1.0,
            rate_limit_rad_per_sec=math.radians(30.0),
            nominal_dt=0.1,
        )
    )
    protector.apply(0.0, 0.1)
    metrics = protector.apply(math.radians(20.0), 0.1)

    assert metrics.rate_limited is True
    assert metrics.final_angle_rad < math.radians(10.0)


def test_continuous_jitter_is_smoothed():
    protector = SteeringProtector(
        SteeringProtectionConfig(max_angle_rad=math.radians(30.0), smoothing_alpha=0.2)
    )
    sequence = [math.radians(v) for v in [8, -8, 8, -8, 8, -8]]
    outputs = [protector.apply(v, 0.1).final_angle_rad for v in sequence]

    input_swings = [abs(sequence[i] - sequence[i - 1]) for i in range(1, len(sequence))]
    output_swings = [abs(outputs[i] - outputs[i - 1]) for i in range(1, len(outputs))]
    assert max(output_swings) < max(input_swings)


def test_short_missing_path_hold_can_be_simulated_with_last_path_buffer():
    buffer = DelayedPathBuffer(delay_frames=0)
    initial = [(0.0, 0.0), (1.0, 0.0)]
    assert buffer.push(initial) == initial
    assert buffer.last_path == initial


def test_long_missing_path_requires_external_fallback_policy():
    delta = mean_abs_path_delta([(0.0, 0.0), (1.0, 0.0)], [(0.0, 2.0), (1.0, 2.0)])
    assert delta > 1.0


def test_fixed_delay_buffer_returns_older_frame_after_warmup():
    buffer = DelayedPathBuffer(delay_frames=2)
    out0 = buffer.push([(0.0, 0.0)])
    out1 = buffer.push([(1.0, 0.0)])
    out2 = buffer.push([(2.0, 0.0)])

    assert out0 == [(0.0, 0.0)]
    assert out1 == [(1.0, 0.0)]
    assert out2 == [(0.0, 0.0)]
