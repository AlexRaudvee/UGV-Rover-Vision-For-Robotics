import math
import pytest
from l2s3_curling.path_follow import command


def test_forward_and_left_turn():
    v,w,status = command((0.,0.,0.), [(0.,0.),(.5,0.),(1.,0.)])
    assert v > 0 and w == pytest.approx(0.) and status == 'following'
    v,w,status = command((0.,0.,0.), [(0.,0.),(.5,.3),(1.,.6)])
    assert v > 0 and w > 0 and status == 'following'


def test_arrival_and_unsafe_inputs_stop():
    assert command((.95,0.,0.), [(0.,0.),(1.,0.)]) == (0.,0.,'arrived')
    assert command((0.,0.,0.), [])[:2] == (0.,0.)
    assert command((math.nan,0.,0.), [(0.,0.),(1.,0.)])[:2] == (0.,0.)


def test_target_behind_commands_slow_turn_instead_of_driving_straight_away():
    v, w, state = command((0., 0., 0.), [(0., 0.), (-1., 0.)], max_speed=.05, max_turn=.25)
    assert 0 < v <= .01 and abs(w) == pytest.approx(.25) and state == 'following'
