import math
import sys
import time
from types import SimpleNamespace as NS
import pytest
from contact_speed import ApproachSpeedController,projected_gap
from motion_config import ContactSettings,DRIVE_ACCELERATION
from route_metrics import MotionMetrics
from grid_map import load_config
from object_tracker import ObjectTracker
from pose_provider import WorldPose
from contact_goal import touch


def test_gap_accounts_for_rotated_box_support_and_robot_front():
    cfg=load_config()
    target={'center':[2,0],'size':[1.4,1.4,.8],'body_size':[1,.4,.8],'yaw':math.pi/4}
    gap=projected_gap(cfg,(0,0,0),target)
    assert gap==pytest.approx(2-.31-.7/math.sqrt(2))
    # Front/side projection makes a misaligned robot slow down earlier.
    assert projected_gap(cfg,(0,0,.1),target)<gap
    assert projected_gap(cfg,(0,0,math.pi),target)<gap


@pytest.mark.parametrize('value',[0,-1,math.inf,math.nan,'fast',True])
def test_invalid_contact_setting_is_rejected(value):
    with pytest.raises(ValueError,match='finite and positive'):
        ContactSettings.from_config({'contact_approach':{'max_speed':value}})


def test_settings_reject_impossible_ramps_and_unknown_names():
    for settings in [{'touch_speed':.5},{'deceleration':1.1},{'acceleration':1.1},
                     {'max_speeed':.4}]:
        with pytest.raises(ValueError):
            ContactSettings.from_config({'contact_approach':settings})


def test_speed_ramps_and_respects_vehicle_limit():
    settings=ContactSettings()
    controller=ApproachSpeedController(settings,.25)
    controller.reset(0)
    last=0.
    for i in range(1,31):
        speed=controller.step(2,last,i*.05)
        assert 0<=speed<=.25
        assert speed-last<=settings.acceleration*.05+1e-10
        last=speed
    assert last==pytest.approx(.25)
    for i in range(31,61):
        speed=controller.step(0,last,i*.05)
        assert last-speed<=settings.deceleration*.05+1e-10
        last=speed
    assert last==pytest.approx(settings.touch_speed)
    slow=ApproachSpeedController(settings,.05)
    assert slow.max_speed==slow.touch_speed==.05


@pytest.mark.parametrize('next_time',[.21,-.01])
def test_feedback_gaps_or_clock_reset_stop_profile(next_time):
    controller=ApproachSpeedController(ContactSettings(),.7)
    controller.reset(0)
    with pytest.raises(RuntimeError,match='feedback interval|backwards'):
        controller.step(2,0,next_time)


def test_duplicate_stamp_cannot_accelerate_robot():
    controller=ApproachSpeedController(ContactSettings(),.7)
    controller.reset(0)
    speed=controller.step(2,0,.05)
    for _ in range(20):
        assert controller.step(2,0,.05)==speed


@pytest.mark.parametrize('gap,speed',[(math.nan,0),(1,math.inf),(1,-1)])
def test_invalid_speed_feedback_is_rejected(gap,speed):
    controller=ApproachSpeedController(ContactSettings(),.7)
    with pytest.raises(RuntimeError,match='Invalid contact speed feedback'):
        controller.step(gap,speed,0)


def test_delayed_vehicle_model_reaches_contact_at_crawl_speed():
    # 80ms command delay plus the generated vehicle's acceleration limit.
    settings=ContactSettings()
    controller=ApproachSpeedController(settings,.7)
    controller.reset(0)
    pending=[0.]*4
    speed=peak=0.
    remaining=2.15
    dt=.02
    for i in range(1,2001):
        cmd=controller.step(remaining-.01,speed,i*dt)
        pending.append(cmd);desired=pending.pop(0)
        speed=max(speed-DRIVE_ACCELERATION*dt,min(desired,speed+DRIVE_ACCELERATION*dt))
        peak=max(peak,speed)
        remaining-=speed*dt
        if remaining<=0:
            break
    else:
        pytest.fail('No contact in bounded time')
    assert peak>.39
    assert speed<=settings.touch_speed+.03
    assert i*dt<15


def test_simulation_stamps_measure_speed_and_exclude_post_contact_samples():
    metrics=MotionMetrics()
    metrics.observe_pose((0,0,0),(1,0))
    metrics.observe_pose((.04,0,0),(1,100000000))
    assert metrics.last_speed==pytest.approx(.4)
    metrics.observe_pose((.06,0,0),(1,100000000))
    metrics.observe_pose((.08,0,0),(1,200000000))
    metrics.observe_pose((.08,0,0),(1,300000000))
    assert metrics.speed_before((1,210000000))==pytest.approx(.4)
    assert metrics.speed_before((2,0)) is None
    assert metrics.max_speed==pytest.approx(.4)
    with pytest.raises(RuntimeError,match='backwards'):
        metrics.observe_pose((.08,0,0),(1,0))


def test_contact_object_pose_uses_stricter_freshness(monkeypatch):
    monkeypatch.setattr('object_tracker.time.monotonic',lambda:100)
    pose=WorldPose(5,4,.4,0,0,0)
    tracker=ObjectTracker.__new__(ObjectTracker)
    tracker.baseline={'red_box':pose}
    tracker.providers={'red_box':NS(current_state=lambda:pose,received=99.5)}
    tracker.assert_unchanged()
    with pytest.raises(RuntimeError,match='contact object pose missing or stale'):
        tracker.assert_unchanged(max_age=.3)


def test_stale_robot_feedback_stops_contact_before_publish(monkeypatch,capsys):
    monkeypatch.setitem(sys.modules,'rclpy',NS(spin_once=lambda *a,**kw:None))
    monkeypatch.setitem(sys.modules,'geometry_msgs',NS())
    monkeypatch.setitem(sys.modules,'geometry_msgs.msg',NS(Twist=object))
    stopped=[]
    monkeypatch.setattr('contact_goal.stop',lambda pub:stopped.append(pub))
    nav=NS(node=object(),pub=object(),provider=NS(
        current=lambda:(2.25,4,0),stamp=(1,0),received=time.monotonic()-1))
    monitor=NS(hit=False,arm=lambda *args:None)
    with pytest.raises(RuntimeError,match='Contact robot pose missing or stale'):
        touch(nav,load_config(),load_config()['objects'][0],monitor,lambda:None)
    assert stopped==[nav.pub]
    assert 'CONTACT_REACHED' not in capsys.readouterr().out
