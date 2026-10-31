import copy
import math
import pytest
from grid_map import GridMap, load_config
from path_planner import astar, plan
from target_resolver import resolve_target, plan_to_object
from navigator import steering
from pose_provider import PoseProvider
from types import SimpleNamespace as NS


def small_config():
    return {'bounds': [0,0,10,10], 'resolution': 0.1,
            'robot': {'radius':0.3,'margin':0.1}, 'walls':[], 'objects':[]}


def test_detour_and_unreachable():
    cfg = small_config()
    cfg['walls'] = [{'center':[5,3], 'size':[0.2,6,1]}]
    grid = GridMap(cfg)
    route = plan(grid,(2,2),(8,2))
    assert max(p[1] for p in route) > 6.4
    assert all(grid.segment_free(a,b) for a,b in zip(route,route[1:]))
    cfg['walls'][0].update(center=[5,5],size=[0.2,10,1])
    with pytest.raises(ValueError, match='No route'):
        plan(GridMap(cfg),(2,2),(8,2))


def test_inflation_and_invalid_endpoints():
    cfg = small_config()
    cfg['walls'] = [{'center':[5,2.25], 'size':[0.2,4.5,1]},
                    {'center':[5,7.75], 'size':[0.2,4.5,1]}]
    cfg['robot']['radius'] = 0.6
    grid = GridMap(cfg)
    with pytest.raises(ValueError, match='No route'):
        plan(grid,(2,5),(8,5))
    for goal in [(5,2),(-1,0),(10,10),(math.nan,0)]:
        with pytest.raises(ValueError):
            plan(grid,(2,5),goal)


def test_maze_target_and_current_start():
    cfg = load_config()
    grid = GridMap(cfg)
    target = resolve_target('赤いオブジェクトまで行ってください', cfg)
    for start in [cfg['robot']['spawn'][:2],[-5.5,4.5]]:
        route = plan_to_object(grid,tuple(start),target)
        assert route[0] == tuple(start)
        assert math.dist(route[-1],target['center']) > grid.padding
        assert all(grid.segment_free(a,b) for a,b in zip(route,route[1:]))
    for text in ['紫の箱まで','箱まで','赤と紫の箱まで']:
        with pytest.raises(ValueError):
            resolve_target(text,cfg)
    cfg['objects'].append(copy.deepcopy(target))
    with pytest.raises(ValueError):
        resolve_target('赤い箱',cfg)


def test_dead_end_requires_backtracking():
    cfg = small_config()
    cfg['walls'] = [ {'center':[5,3], 'size':[4,0.2,1]},
                     {'center':[5,7], 'size':[4,0.2,1]},
                     {'center':[7,5], 'size':[0.2,4,1]} ]
    grid = GridMap(cfg)
    route = plan(grid,(5,5),(8.5,5))
    assert min(p[0] for p in route) < 3
    assert all(grid.segment_free(a,b) for a,b in zip(route,route[1:]))


def test_steering_rotates_before_translating():
    assert steering((0,0,math.pi),(1,0))[0] == 0
    linear, angular = steering((0,0,0),(1,0))
    assert 0 < linear <= .7
    assert angular == 0
    assert abs(steering((0,0,-math.pi+.01),(-1,0))[1]) < .1


def test_pose_freshness_and_frame_selection(monkeypatch):
    provider = PoseProvider.__new__(PoseProvider)
    provider.model_name = 'vehicle_blue'
    provider.state = None
    provider.error = 'World pose missing'
    provider.pose, provider.received, provider.stamp = None, 0, None
    monkeypatch.setattr('pose_provider.time.monotonic',lambda:100)
    def transform(child, sec):
        return NS(child_frame_id=child, header=NS(frame_id='maze',stamp=NS(sec=sec,nanosec=0)),
                  transform=NS(translation=NS(x=-5,y=-4,z=.325),
                               rotation=NS(x=0,y=0,z=math.sqrt(.5),w=math.sqrt(.5))))
    provider.callback(NS(transforms=[transform('chassis',1)]))
    with pytest.raises(RuntimeError):
        provider.current()
    provider.callback(NS(transforms=[transform('vehicle_blue',1)]))
    assert provider.current() == pytest.approx((-5,-4,math.pi/2))
    monkeypatch.setattr('pose_provider.time.monotonic',lambda:102)
    provider.callback(NS(transforms=[transform('vehicle_blue',1)]))
    with pytest.raises(RuntimeError, match='stale'):
        provider.current()


def test_feedback_reaches_goal_without_crossing_obstacles():
    # Ideal kinematic model is a unit test, not Gazebo validation.
    cfg = load_config()
    grid = GridMap(cfg)
    route = plan_to_object(grid,(-5,-4),cfg['objects'][0])
    pose = (-5.,-4.,0.)
    for goal in route[1:]:
        for _ in range(20000):
            if math.dist(pose[:2],goal) < .025:
                break
            linear, angular = steering(pose,goal)
            dt = .03
            x,y,yaw = pose
            pose = (x+linear*math.cos(yaw)*dt, y+linear*math.sin(yaw)*dt,
                    yaw+angular*dt)
            assert grid.point_free(pose[:2])
        else:
            pytest.fail('Controller did not reach waypoint')
    assert math.dist(pose[:2],route[-1]) < .1


@pytest.mark.parametrize('failure', [RuntimeError('pose stale'), KeyboardInterrupt()])
def test_navigation_stops_on_failure(monkeypatch, failure):
    import sys
    from navigator import Navigator
    fake_ros = NS(spin_once=lambda *args, **kwargs: None)
    monkeypatch.setitem(sys.modules, 'rclpy', fake_ros)
    # ROS message stubs keep this safety test runnable without ROS installed.
    monkeypatch.setitem(sys.modules, 'geometry_msgs', NS())
    monkeypatch.setitem(sys.modules, 'geometry_msgs.msg', NS(Twist=object))
    stopped = []
    monkeypatch.setattr('navigator.stop', lambda pub: stopped.append(pub))
    nav = Navigator.__new__(Navigator)
    nav.node, nav.pub = object(), object()
    def fail():
        raise failure
    nav.provider = NS(current=fail)
    with pytest.raises(type(failure)):
        nav.follow([(0,0),(1,0)])
    assert stopped == [nav.pub]
