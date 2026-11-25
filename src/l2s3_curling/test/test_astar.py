import pytest
from l2s3_curling.astar import plan


def test_routes_around_obstacle_and_changes_when_obstacle_moves():
    w, h = 12, 9
    grid = [0] * (w*h)
    for y in range(2, 7): grid[y*w+5] = 100
    first, reason = plan(grid, w, h, 1., (0, 0), (1.5, 4.5), (10.5, 4.5))
    assert first and not reason
    assert all(grid[int(y)*w+int(x)] < 50 for x, y in first)
    assert any(y < 2 or y >= 7 for _, y in first)
    for y in range(2, 7): grid[y*w+5] = 0
    second, _ = plan(grid, w, h, 1., (0, 0), (1.5, 4.5), (10.5, 4.5))
    assert second and max(abs(y-4.5) for _, y in second) < .01


def test_no_cutting_corners_and_blocked_goal_or_start():
    grid = [0, 100, 100, 0]
    assert not plan(grid, 2, 2, 1, (0, 0), (.5,.5), (1.5,1.5))[0]
    assert 'goal' in plan(grid, 2, 2, 1, (0, 0), (.5,.5), (1.5,.5))[1]
    assert 'start' in plan(grid, 2, 2, 1, (0, 0), (1.5,.5), (.5,.5))[1]


def test_unknown_policy_is_explicit():
    assert plan([0, -1, 0], 3, 1, 1, (0,0), (.5,.5), (2.5,.5), True)[0]
    assert not plan([0, -1, 0], 3, 1, 1, (0,0), (.5,.5), (2.5,.5), False)[0]


def test_invalid_grid_rejected():
    with pytest.raises(ValueError): plan([], 3, 1, 1, (0,0), (0,0), (1,0))
