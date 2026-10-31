import math
import sys
from types import SimpleNamespace as NS
import pytest
from grid_map import GridMap, load_config
from object_tracker import ObjectTracker
from scene_helpers import runtime_config
from pose_provider import PoseProvider, WorldPose
from target_resolver import plan_to_object


def pose(x=-5, y=3, z=.4, roll=0, pitch=0, yaw=0):
    return WorldPose(x,y,z,roll,pitch,yaw)


def test_moved_object_replaces_old_occupancy_and_goal():
    original = load_config()
    live = runtime_config(original, {'red_box':pose()})
    assert original['objects'][0]['center'] == [5,4]
    assert live['objects'][0]['center'] == [-5,3]
    before, after = GridMap(original), GridMap(live)
    assert not before.point_free((5,4)) and after.point_free((5,4))
    assert not after.point_free((-5,3))
    route = plan_to_object(after,(-5,-4),live['objects'][0])
    assert math.dist(route[-1],(-5,3)) < 3
    assert route[-1] != (2.25,4)
    assert all(after.segment_free(a,b) for a,b in zip(route,route[1:]))


def test_rotated_rectangle_is_conservatively_enclosed():
    cfg = load_config()
    cfg['objects'][0]['size'] = [1,.4,.8]
    live = runtime_config(cfg, {'red_box':pose(yaw=math.pi/4)})
    obj = live['objects'][0]
    assert obj['size'][:2] == pytest.approx([1.4/math.sqrt(2)]*2)
    for x in [-.5,.5]:
        for y in [-.2,.2]:
            rx, ry = (x-y)/math.sqrt(2), (x+y)/math.sqrt(2)
            assert abs(rx) <= obj['size'][0]/2+1e-10
            assert abs(ry) <= obj['size'][1]/2+1e-10


@pytest.mark.parametrize('state,reason', [
    (pose(x=20),'outside'), (pose(x=0,y=-3),'overlaps'),
    (pose(z=1),'ground'), (pose(roll=.1),'upright'),
    (pose(pitch=.1),'upright'), (pose(x=math.nan),'non-finite')])
def test_invalid_object_placement_is_rejected(state,reason):
    with pytest.raises(ValueError,match=reason):
        runtime_config(load_config(), {'red_box':state})


def test_moved_object_with_no_approach_is_rejected():
    cfg=load_config();cfg['bounds']=[-8,-8,8,8]  # Explicitly sealed legacy field.
    cfg = runtime_config(cfg, {'red_box':pose(x=1,y=-3)})
    with pytest.raises(ValueError,match='No reachable'):
        plan_to_object(GridMap(cfg),(-5,-4),cfg['objects'][0])


def tracker_with(state, baseline=None):
    tracker = ObjectTracker.__new__(ObjectTracker)
    tracker.baseline = {'red_box':baseline or pose()}
    tracker.providers = {'red_box':NS(current_state=lambda:state)}
    return tracker


@pytest.mark.parametrize('state', [pose(x=-4.94),pose(yaw=.06),pose(z=.46)])
def test_guard_detects_movement(state):
    with pytest.raises(RuntimeError,match='moved during navigation'):
        tracker_with(state).assert_unchanged()


def test_guard_uses_wrapped_yaw_and_checks_freshness():
    tracker_with(pose(yaw=-math.pi+.01),pose(yaw=math.pi-.01)).assert_unchanged()
    tracker = tracker_with(pose())
    def stale():
        raise RuntimeError('red_box: World pose missing or stale')
    tracker.providers['red_box'].current_state = stale
    with pytest.raises(RuntimeError,match='stale'):
        tracker.assert_unchanged()


def test_snapshot_does_not_fall_back_to_initial_or_previous_position(monkeypatch):
    monkeypatch.setitem(sys.modules,'rclpy',NS(spin_once=lambda *a,**kw:None))
    monkeypatch.setattr('object_tracker.time.monotonic',lambda:100)
    tracker = tracker_with(pose())
    tracker.node = object()
    tracker.config = load_config()
    tracker.providers['red_box'].state = pose()
    tracker.providers['red_box'].received = 99
    with pytest.raises(RuntimeError,match='No fresh object'):
        tracker.snapshot(timeout=0)


def test_pose_provider_rejects_bad_frame_and_invalid_data(monkeypatch):
    provider = PoseProvider.__new__(PoseProvider)
    provider.model_name = 'red_box'
    provider.pose = provider.state = provider.stamp = None
    provider.received = 0
    provider.error = 'World pose missing'
    monkeypatch.setattr('pose_provider.time.monotonic',lambda:100)
    def message(frame='maze', x=-5, w=1, stamp=1):
        return NS(transforms=[NS(child_frame_id='red_box',
                  header=NS(frame_id=frame,stamp=NS(sec=stamp,nanosec=0)),
                  transform=NS(translation=NS(x=x,y=3,z=.4),
                               rotation=NS(x=0,y=0,z=0,w=w)))])
    provider.callback(message())
    assert provider.current_state() == pose()
    provider.callback(message(frame='vehicle_blue'))
    with pytest.raises(RuntimeError,match='world frame'):
        provider.current_state()
    for msg in [message(x=math.inf,stamp=2),message(w=0,stamp=3)]:
        provider.callback(msg)
        with pytest.raises(RuntimeError,match='Invalid'):
            provider.current_state()


def test_movement_guard_failure_sends_stop(monkeypatch):
    from navigator import Navigator
    monkeypatch.setitem(sys.modules,'rclpy',NS(spin_once=lambda *a,**kw:None))
    monkeypatch.setitem(sys.modules,'geometry_msgs',NS())
    monkeypatch.setitem(sys.modules,'geometry_msgs.msg',NS(Twist=object))
    stopped = []
    monkeypatch.setattr('navigator.stop',lambda pub:stopped.append(pub))
    nav = Navigator.__new__(Navigator)
    nav.node, nav.pub = object(),object()
    with pytest.raises(RuntimeError,match='moved'):
        nav.follow([(0,0),(1,0)],guard=tracker_with(pose(x=-4.9)).assert_unchanged)
    assert stopped == [nav.pub]


def test_object_moved_while_stopping_does_not_report_arrival(monkeypatch,capsys):
    from navigator import Navigator
    monkeypatch.setitem(sys.modules,'rclpy',NS(spin_once=lambda *a,**kw:None))
    monkeypatch.setitem(sys.modules,'geometry_msgs',NS())
    monkeypatch.setitem(sys.modules,'geometry_msgs.msg',NS(Twist=object))
    stopped = []
    monkeypatch.setattr('navigator.stop',lambda pub:stopped.append(pub))
    nav = Navigator.__new__(Navigator)
    nav.node,nav.pub = object(),object()
    nav.provider = NS(current=lambda:(1,0,0))
    nav.grid = NS(clearance=lambda point:1,point_free=lambda point:True)
    checks = []
    def guard():
        checks.append(True)
        if len(checks)>1:
            raise RuntimeError('Object moved during final stop')
    with pytest.raises(RuntimeError,match='final stop'):
        nav.follow([(0,0),(1,0)],guard=guard)
    assert len(stopped) == 2
    assert 'ARRIVED' not in capsys.readouterr().out
