"""Map-directed camera search without hardware."""
import math

from l2s2_localisation.pan_tracking import PanController
from l2s2_localisation.relocalization_search import RelocalizationSearch


def test_one_tag_is_held_then_camera_leaves_it_for_another():
    controller = PanController([1, 2], one_marker='search')
    controller.observe({1: 0.}, 0.)
    controller.tracked_ids = [1]
    search = RelocalizationSearch(100., 'tag', 120.)
    pose = (0., 0., 0.)
    markers = {1: (1., 0.), 2: (0., 1.)}
    assert search.step(controller, 0., pose, markers, {}) == (0., 'relocalize_hold')
    target, mode = search.step(controller, 1.1, pose, markers, {'tag:1': 100.5})
    assert target == 90. and mode == 'relocalize_search'
    assert search.target_id == 2
    controller.observe({2: 5.}, 1.2)
    controller.tracked_ids = [2]
    target, mode = search.step(controller, 1.2, pose, markers, {'tag:1': 100.5})
    assert mode == 'relocalize_hold' and math.isfinite(target)


def test_two_tags_hold_at_midpoint_then_select_unaccepted_tag():
    controller = PanController([1, 2, 3])
    controller.observe({1: -20., 2: 20.}, 0.)
    controller.tracked_ids = [1, 2]
    search = RelocalizationSearch(100., 'tag', 120.)
    markers = {1: (1., -.3), 2: (1., .3), 3: (0., 1.)}
    assert search.step(controller, 0., (0., 0., 0.), markers, {}) == (0., 'relocalize_hold')
    target, mode = search.step(controller, 1.1, (0., 0., 0.), markers, {})
    assert mode == 'relocalize_search' and target == 90.


def test_no_markers_prefers_last_bearing_then_map_and_fallback_sweep():
    controller = PanController([1, 2])
    search = RelocalizationSearch(100., 'tag', 120., last_bearing=-20.)
    assert search.step(controller, 0., (0., 0., 0.), {2: (0., 1.)}, {}) == (-20., 'relocalize_last_tag')
    controller.pan = -20.
    assert search.step(controller, 1., (0., 0., 0.), {2: (0., 1.)}, {}) == (90., 'relocalize_search')
    controller.pan = 90.
    assert search.step(controller, 2.1, (0., 0., 0.), {2: (0., 1.)}, {})[1] == 'relocalize_search'
    assert search.step(controller, 3.2, (0., 0., 0.), {2: (0., 1.)}, {})[1] == 'relocalize_sweep'
    assert search.step(controller, 3.3, None, {}, {})[1] == 'relocalize_sweep'
