import numpy as np
import pytest

from l2s3_curling.motion import MotionLimiter
from l2s3_curling.planner import turn_direction_mismatch


def test_turn_sign_fault_only_checks_consistent_in_place_turns():
    assert turn_direction_mismatch((0., .5), (0., .5), -.2)
    assert not turn_direction_mismatch((.2, .5), (.2, .5), -.2)  # curved drive
    assert not turn_direction_mismatch((0., .5), (0., -.5), -.2)  # changing direction
    assert not turn_direction_mismatch((0., .5), (0., 0.), -.2)  # stopping
    assert not turn_direction_mismatch((0., .5), (0., .5), -.03)  # gyro noise
    assert not turn_direction_mismatch((0., .5), (0., .5), .2)


def test_flickering_clear_frames_do_not_cause_motion_pulses():
    limiter=MotionLimiter()
    for i in range(40):
        assert limiter.update(.1 if i%3 else 0.,0.,i*.1)==(0.,0.)


def test_acceleration_is_bounded_and_obstacle_stop_is_immediate():
    limiter=MotionLimiter();previous=np.zeros(2)
    for i in range(20):
        command=np.array(limiter.update(.1,.4,i*.1))
        assert np.all(command-previous <= np.array([.15,.6])*.1+1e-9)
        assert np.all(command>=0)
        previous=command
    np.testing.assert_allclose(command,[.1,.4])
    assert limiter.update(0.,0.,2.)==(0.,0.)
    assert limiter.update(.1,0.,2.1)==(0.,0.)
    assert limiter.update(.1,0.,2.3)==(0.,0.)
    assert limiter.update(.1,0.,2.6)[0]>0


def test_turn_request_does_not_continue_forward_and_long_gap_rearms():
    limiter=MotionLimiter()
    for i in range(15):limiter.update(.1,0.,i*.1)
    assert limiter.update(0.,.4,1.5)[0]==0.
    assert limiter.update(.1,0.,3.)==(0.,0.)
    assert limiter.update(float('nan'),0.,3.1)==(0.,0.)
    with pytest.raises(ValueError):MotionLimiter(linear_acceleration=0.)
