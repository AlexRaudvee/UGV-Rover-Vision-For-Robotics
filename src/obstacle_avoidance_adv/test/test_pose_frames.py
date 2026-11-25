# Provides pi for expressing a 90-degree heading in radians.
import math

# Provides the numerical comparison used to check the returned pose.
import numpy as np

# Allows the same test function to run with several different inputs.
import pytest

# Import the coordinate-conversion function that we want to test.
from obstacle_avoidance_adv.planner import relative_pose


# Run the test once for each row below.
# Each row supplies: current pose, assumed starting pose, expected relative pose.
@pytest.mark.parametrize('pose, origin, expected', [
    (
        (-3.2, 0., 0.),  # Current field pose: exactly at the assumed start.
        (-3.2, 0., 0.),  # Assumed starting pose.
        (0., 0., 0.),    # Expected result: no displacement or heading change.
    ),
    (
        (-2.2, 0., 0.),  # Current field pose: one metre along +x from the start.
        (-3.2, 0., 0.),  # Assumed starting pose.
        (1., 0., 0.),    # Expected result: one metre forward, still facing +x.
    ),
    (
        (-2.2, .5, math.pi / 2),  # Current pose: displaced and facing field +y.
        (-3.2, 0., 0.),          # Assumed starting pose, still facing field +x.
        (1., .5, math.pi / 2),   # Expected: x=1 m, y=0.5 m, heading=90 degrees.
    ),
])
def test_relative_pose(pose, origin, expected):
    # Calculate the relative pose for the current test case.
    actual = relative_pose(pose, origin)

    # Check all three output values against the expected values.
    np.testing.assert_allclose(
        actual,       # Values returned by our function.
        expected,     # Values that the function should return.
        rtol=0.,      # Disable tolerance proportional to the expected value.
        atol=1e-9,    # Allow tiny floating-point differences up to 0.000000001.
    )
