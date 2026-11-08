"""Pair delayed sensor frames without manufacturing missing measurements."""

from types import SimpleNamespace

from detection_improved.synchronization import select_pair


def message(seconds):
    return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(
        sec=int(seconds), nanosec=round((seconds % 1) * 1e9))))


def test_pair_delayed_range_with_previous_rgb_before_unmatched_new_rgb():
    frames = [(message(10.), 20., 1), (message(10.2), 20.2, 2)]
    measurements = [(message(10.), 20.15)]
    assert select_pair(frames, measurements, 0, 20.21, 1., .12)[2] == 1
    assert select_pair(frames, measurements, 1, 20.21, 1., .12) is None
    measurements.append((message(10.2), 20.3))
    assert select_pair(frames, measurements, 1, 20.31, 1., .12)[2] == 2


def test_missing_range_has_bounded_wait_but_no_range_fallback():
    frames = [(message(10.), 20., 1), (message(10.2), 20.2, 2)]
    assert select_pair(frames, [], 0, 20.21, 1., .12) is None
    assert select_pair(frames, [], 0, 20.3, 1., .12)[1] is None
    assert select_pair(frames, [], 0, 21.3, 1., .12) is None


def test_zero_stamps_and_old_receipts_do_not_match():
    frames = [(message(10.), 20., 1)]
    for sensor, received in [(message(10.), 18.), (message(9.7), 20.), (message(0.), 20.)]:
        assert select_pair(frames, [(sensor, received)], 0, 20.1, 1., .12) is None


def test_depth_delayed_past_arrival_grace_can_still_match_buffered_rgb():
    frames = [(message(10.), 20., 1), (message(10.2), 20.2, 2)]
    waiting = select_pair(frames, [], 0, 20.3, 1., .12)
    assert waiting[1] is None
    # The adapter must not mark RGB processed before an actual pair is available.
    delayed = select_pair(frames, [(message(10.), 20.4)], 0, 20.41, 1., .12)
    assert delayed[1] is not None and delayed[2] == 1
