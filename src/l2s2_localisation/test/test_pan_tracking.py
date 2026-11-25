"""Behavioral tests for smooth multi-marker tracking, independent of ROS/hardware."""
import pytest

from l2s2_localisation.pan_tracking import PanController


def test_short_dropouts_keep_tracking_but_third_miss_centers():
    c = PanController([2, 3], alpha=1)
    c.pan = 20
    c.observe({2: 10, 3: 20}, 0)
    c.observe({2: 10}, .1)
    assert c.step(.05, .1) > 20
    c.observe({}, .2)
    before = c.pan
    assert c.step(.05, .2) > before
    assert c.mode == 'tracking_markers'
    c.observe({2: 10}, .3)
    before = c.pan
    assert c.step(.05, .3) < before
    assert c.mode == 'centering'


def test_reappearance_resets_consecutive_misses():
    c = PanController([2, 3])
    for frame, tags in enumerate(({2: 20, 3: 40}, {2: 20}, {2: 20, 3: 40}, {}, {})):
        c.observe(tags, frame*.1)
        c.step(.05, frame*.1)
        assert c.mode == 'tracking_markers'


def test_speed_is_bounded_in_both_directions_and_after_a_timer_stall():
    c = PanController([2, 3], speed=8)
    c.observe({2: 60, 3: 80}, 0)
    assert c.step(.05, .05) == pytest.approx(.4)
    assert c.step(10, .1) == pytest.approx(1.2)
    c.tracks.clear()
    c.pan = c.limit
    assert c.step(.05, .15) == pytest.approx(c.limit - .4)


def test_one_tag_centers_by_default_and_expired_tags_trigger_a_sweep():
    c = PanController([2, 3])
    c.pan = 2
    c.observe({2: 30}, 0)
    assert c.step(.1, .1) < 2
    c.observe({2: 20, 3: 40}, .2)
    c.step(.05, .2)
    assert c.mode == 'tracking_markers'
    before = c.pan
    assert c.step(.05, 1.1) > before
    assert c.mode == 'searching_without_markers'


def test_zero_tags_sweeps_both_limits_until_a_tag_reappears():
    c = PanController([2, 3], limit=4., speed=8., one_marker='search', search_reach=2.)
    pans = [c.step(.1, frame*.1) for frame in range(18)]
    assert c.mode == 'searching_without_markers'
    assert max(pans) == pytest.approx(4.)
    assert min(pans) == pytest.approx(-4.)
    assert pans[-1] > pans[-3]
    c.observe({2: -3.}, 1.8)
    assert c.step(.1, 1.8) < pans[-1]
    assert c.mode == 'searching_with_one_marker'
    c.observe({2: -3., 3: 12.}, 1.9)
    c.step(.1, 1.9)
    assert c.mode == 'tracking_markers'


def test_deadband_stops_noise_and_smoothing_reduces_a_bad_frame():
    c = PanController([2, 3], deadband=3, alpha=.25)
    for frame in range(20):
        noise = (-1)**frame * 2
        c.observe({2: -10+noise, 3: 10+noise}, frame*.02)
        assert c.step(.02, frame*.02) == 0
    c.observe({2: 70, 3: 90}, .5)
    assert c.tracks[2]['angle'] < 20


def test_image_exposure_pan_compensates_for_camera_motion():
    c = PanController([2, 3], alpha=1)
    c.pan = 20
    c.observe({2: -10, 3: 10}, 0, exposure_pan=20)
    assert c.step(.05, 0) == 20
    # A delayed image taken at 15 degrees must not be treated as taken at 20.
    c.observe({2: -5, 3: 15}, .1, exposure_pan=15)
    assert c.step(.05, .1) == 20


def test_mechanical_limit_and_reacquisition_after_expiry():
    c = PanController([2, 3], speed=100, alpha=.25, limit=30)
    for frame in range(10):
        c.observe({2: 50, 3: 70}, frame*.05)
        assert c.step(.05, frame*.05) <= 30
    c.observe({2: -10, 3: 10}, 2, exposure_pan=0)
    assert c.tracks[2]['angle'] == -10, 'expired angles contaminated reacquisition'


@pytest.mark.parametrize('settings', [{'missing_frames': 0}, {'alpha': 0}, {'speed': -1}, {'speed': float('nan')}])
def test_invalid_settings(settings):
    with pytest.raises(ValueError):
        PanController([2, 3], **settings)


@pytest.mark.parametrize('ids', [[], [1], [1, 1], [-1, 2], [1, '2'], [True, 2]])
def test_invalid_id_lists(ids):
    with pytest.raises(ValueError):
        PanController(ids)


@pytest.mark.parametrize('visible', [{0: 10, 5: 30}, {1: 10, 4: 30}, {2: 10, 3: 30}])
def test_any_two_of_six_can_start_tracking(visible):
    c = PanController(list(range(6)), alpha=1)
    c.observe(visible, 0)
    assert c.step(.05, .05) > 0
    assert c.mode == 'tracking_markers'
    assert set(c.tracked_ids) == set(visible)


def test_center_outermost_markers_instead_of_biasing_toward_a_cluster():
    c = PanController(list(range(6)), alpha=1)
    c.observe(dict(enumerate([-50, -45, -40, -35, 20, 50])), 0)
    assert c.step(.05, .05) == 0
    assert c.tracked_ids == list(range(6))
    assert c.mode == 'tracking_markers'


def test_losing_one_of_three_does_not_center_while_two_others_remain():
    c = PanController(list(range(6)), alpha=1)
    c.pan = 10
    c.observe({0: -20, 2: 10, 5: 30}, 0)
    for frame in range(1, 4):
        c.observe({2: 10, 5: 30}, frame*.1)
        c.step(.05, frame*.1)
        assert c.mode == 'tracking_markers'
    assert c.tracked_ids == [2, 5]
    for frame in range(4, 7):
        c.observe({5: 30}, frame*.1)
        c.step(.05, frame*.1)
    assert c.tracked_ids == [5]
    assert c.mode == 'centering'


def test_tag_handover_and_time_expiry_apply_independently():
    c = PanController(list(range(6)), alpha=1)
    c.observe({0: 10, 1: 30}, 0)
    c.observe({1: 10, 4: 30, 99: 80}, .7)
    c.step(.05, .85)
    assert c.mode == 'tracking_markers'
    assert c.tracked_ids == [1, 4], 'old or unconfigured tags remained active'
