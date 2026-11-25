"""Default one-marker search and reacquisition behaviour of the pan tracker."""
import pytest

from l2s2_localisation.pan_tracking import PanController

HALF_FOV = 48.0      # USB camera: about 96 deg horizontal


def run(controller, tags, seconds, dt=.05):
    """Robot standing still; tags = {id: angle from the robot}. Only tags in the image are seen."""
    history, now = [], 0.
    for _ in range(int(seconds/dt)):
        now += dt
        visible = {tag: angle-controller.pan for tag, angle in tags.items()
                   if abs(angle-controller.pan) < HALF_FOV}
        controller.observe(visible, now)
        history.append((controller.step(dt, now), controller.mode, set(visible)))
    return history


def test_default_searches_instead_of_returning_to_zero_with_one_tag():
    c = PanController([2, 3])
    history = run(c, {2: 30.}, 5)
    assert history[-1][1] == 'searching_with_one_marker'
    assert c.pan > 30.
    assert all(2 in visible for _, _, visible in history)


def test_search_keeps_the_tag_in_view_and_looks_both_ways():
    c = PanController([2, 3], search_reach=36.)
    history = run(c, {2: 20.}, 40)
    assert all(2 in visible for _, _, visible in history[5:])
    pans = [pan for pan, _, _ in history]
    assert max(pans) > 20+30 and min(pans) < 20-30
    assert history[-1][1] == 'searching_with_one_marker'


def test_search_finds_a_second_tag_and_goes_back_to_tracking():
    c = PanController([2, 3], search_reach=36.)
    history = run(c, {2: 20., 3: 100.}, 40)
    found = next(i for i, (_, _, visible) in enumerate(history) if visible == {2, 3})
    assert history[found+5][1] == 'tracking_markers'
    assert history[-1][1] == 'tracking_markers'


def test_search_stays_inside_the_pan_limit():
    c = PanController([2, 3], search_reach=36., limit=60.)
    history = run(c, {2: 50.}, 30)
    assert max(pan for pan, _, _ in history) <= 60.
    assert min(pan for pan, _, _ in history) < 50-30


def test_no_tag_still_turns_back_to_zero():
    c = PanController([2, 3])
    c.pan = 40.
    history = run(c, {}, 10)
    assert history[-1][1] == 'centering' and c.pan == 0.


@pytest.mark.parametrize('settings', [dict(search_reach=-1.), dict(search_reach=90.),
                                    dict(search_reach=float('nan'))])
def test_invalid_settings(settings):
    with pytest.raises(ValueError):
        PanController([2, 3], **settings)
