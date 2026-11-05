"""Timestamp pairing under delayed depth, missing frames, and lost streams."""

from types import SimpleNamespace

from obstacle_detection.synchronization import select_rgb


def message(seconds, frame='optical', width=640):
    return SimpleNamespace(
        width=width, height=480,
        header=SimpleNamespace(frame_id=frame, stamp=SimpleNamespace(
            sec=int(seconds), nanosec=round((seconds % 1) * 1e9))))


def test_delayed_depth_selects_previous_unprocessed_rgb_instead_of_newest_unmatched():
    frames = [(message(10.0), 20.0, 1), (message(10.2), 20.2, 2)]
    depths = [(message(10.0), 20.15)]
    assert select_rgb(frames, depths, 0, 20.21, 1.0, 0.12, True)[2] == 1
    assert select_rgb(frames, depths, 1, 20.21, 1.0, 0.12, True) is None
    depths.append((message(10.2), 20.3))
    assert select_rgb(frames, depths, 1, 20.31, 1.0, 0.12, True)[2] == 2


def test_missing_depth_wait_is_bounded_even_when_new_rgb_keeps_arriving():
    frames = [(message(10.0), 20.0, 1), (message(10.2), 20.2, 2)]
    assert select_rgb(frames, [], 0, 20.21, 1.0, 0.12, True) is None
    assert select_rgb(frames, [], 0, 20.3, 1.0, 0.12, True)[2] == 2
    assert select_rgb(frames, [], 0, 20.21, 1.0, 0.12, False)[2] == 2


def test_wrong_geometry_timestamp_or_stale_depth_does_not_form_a_pair():
    frames = [(message(10.0), 20.0, 1)]
    for depth, received in ((message(10.0, frame='wrong'), 20.0),
                            (message(10.0, width=320), 20.0),
                            (message(9.7), 20.0),
                            (message(10.0), 18.0)):
        assert select_rgb(frames, [(depth, received)], 0, 20.1, 1.0, 0.12, True) is None
    assert select_rgb(frames, [], 0, 21.1, 1.0, 0.12, True) is None
    assert select_rgb(frames, [], 1, 20.3, 1.0, 0.12, True) is None
