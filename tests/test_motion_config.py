import math
import pytest
from motion_config import MotionSettings
from navigator import steering


def test_controller_obeys_custom_limits_and_slows_near_goal():
    settings = MotionSettings.from_config({'navigation': {
        'max_linear_speed':.45,'max_angular_speed':.6,
        'distance_gain':1.0,'heading_gain':2.0}})
    assert steering((0,0,0),(10,0),settings) == (.45,0)
    assert steering((0,0,0),(.05,0),settings) == (.05,0)
    linear, angular = steering((0,0,0),(0,10),settings)
    assert linear == 0 and angular == .6
    linear, angular = steering((0,0,0),(0,-10),settings)
    assert linear == 0 and angular == -.6
    # JSON integer limits must still produce floats accepted by ROS Twist.
    integer_limits = MotionSettings(max_linear_speed=1, max_angular_speed=1)
    assert all(isinstance(v,float) for v in steering((0,0,0),(10,0),integer_limits))


@pytest.mark.parametrize('value',[0,-1,math.inf,math.nan,'fast'])
def test_invalid_speed_is_rejected(value):
    with pytest.raises(ValueError,match='finite and positive'):
        MotionSettings.from_config({'navigation':{'max_linear_speed':value}})


def test_typo_is_not_silently_ignored():
    with pytest.raises(ValueError,match='Unknown navigation'):
        MotionSettings.from_config({'navigation':{'max_speed':1}})
