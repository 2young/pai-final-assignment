import math
import pytest
from footprint import FootprintScene,integrate
from pose_planner import pose_plan,PoseRoute,PoseSettings
from adaptive_planner import adaptive_plan
from planning_limits import PlanningBudget,PlanningSettings,PlanningLimitError
from grid_map import load_config,GridMap
from scene_helpers import runtime_config
from pose_provider import WorldPose
from path_geometry import ArcPath
from pure_pursuit import PursuitController,stopping_trajectory_free
from motion_config import MotionSettings
from follower_config import FollowerSettings


def corridor():
    return {'bounds':[0,0,14,10],'resolution':.1,
            'robot':{'radius':1.2,'margin':.1,'footprint_bounds':[-1.,-.3,.3,.3]},
            'walls':[{'id':'lower','center':[7,3.9],'size':[14,.2,1]},
                     {'id':'upper','center':[7,6.1],'size':[14,.2,1]}],
            'objects':[],'planner_mode':'hybrid',
            'navigation':{'max_linear_speed':1.4,'max_angular_speed':1.},
            'path_following':{'tracking_margin':.1}}


def test_orientation_opens_corridor_without_shrinking_margins():
    cfg=corridor();scene=FootprintScene(cfg)
    assert scene.free((3,5,0),extra=.1)
    assert not scene.free((3,5,math.pi/2),extra=.1)
    assert not GridMap(cfg).planning_view().point_free((3,5))
    g,route=adaptive_plan(cfg,(3,5),goal=(11,5),start_yaw=0)
    assert isinstance(route,PoseRoute)
    assert all(g.motion_free(m.start,m.v,m.w,m.duration,extra=.1) for m in route.motions)
    assert cfg['robot']['margin']==.1 and cfg['path_following']['tracking_margin']==.1


def test_reverse_connector_exits_slot_where_pivot_is_unsafe():
    cfg=corridor()
    _,route=adaptive_plan(cfg,(8,5),goal=(3,5),start_yaw=0)
    assert route.motions and all(m.v<0 for m in route.motions)
    assert route[-1]==pytest.approx((3,5))
    assert route.poses[-1][2]%(2*math.pi)==pytest.approx(0,abs=1e-8)


def test_sweep_checks_rotating_tail_and_thin_obstacle_between_endpoints():
    cfg=corridor();cfg['walls']=[{'id':'pin','center':[4.8,3.8],'size':[.1,.1,1]}]
    cfg['robot']['footprint_bounds']=[-2.,-.3,.4,.3]
    scene=FootprintScene(cfg)
    a,b=(6,5,0),(6,5,math.pi/2)
    assert scene.free(a) and scene.free(b)
    assert not scene.sweep(a,b)
    assert not scene.motion_free(a,0,1,math.pi/2)
    cfg['walls']=[{'id':'thin','center':[6,5],'size':[.001,8,1]}]
    scene=FootprintScene(cfg)
    assert scene.free((3,5,0)) and scene.free((10,5,0))
    assert not scene.sweep((3,5,0),(10,5,0))


def test_rear_braking_tail_includes_reverse_motion_after_zero_command():
    cfg=corridor();cfg['walls']=[{'id':'rear_wall','center':[1.5,5],'size':[.1,8,1]}]
    scene=FootprintScene(cfg);virtual=(3,5,math.pi)
    assert scene.free((3,5,0))
    assert not stopping_trajectory_free(scene,virtual,0,0,1,0,FollowerSettings(),scene,-1)
    assert stopping_trajectory_free(scene,virtual,0,0,0,0,FollowerSettings(),scene,-1)


@pytest.mark.parametrize('direction',[1,-1])
def test_pose_follower_keeps_straight_speed_with_lag_and_safe_sweeps(direction):
    cfg=corridor();scene=FootprintScene(cfg)
    start,goal=((3,5),(11,5)) if direction>0 else ((11,5),(3,5))
    physical=(*start,0.);v=w=0.;maximum=0.
    motion=MotionSettings(max_linear_speed=1.4 if direction>0 else .7,max_angular_speed=1.)
    controller=PursuitController(scene,ArcPath([start,goal]),motion,FollowerSettings(),scene,direction)
    for step in range(2000):
        virtual=(*physical[:2],physical[2]+(math.pi if direction<0 else 0))
        cv,cw,done=controller.command(virtual,step*.03)
        if done:break
        desired=direction*cv
        v+=max(-.03,min(.03,desired-v));w+=max(-.06,min(.06,cw-w))
        assert scene.motion_free(physical,v,w,.03)
        physical=integrate(physical,v,w,.03);maximum=max(maximum,abs(v))
    else:pytest.fail('Pose follower did not converge')
    assert math.dist(physical[:2],goal)<.1
    assert maximum>motion.max_linear_speed*.95


def reported_scene():
    return runtime_config(load_config(),{
        'dead_end':WorldPose(-2,2.7871241569519043,.5,0,0,0),
        'west_wall':WorldPose(4.379763603210449,0,.5,0,0,0),
        'southeast_block':WorldPose(3.735650062561035,-8.035699844360352,.5,0,0,0),
        'red_box':WorldPose(2.4746155738830566,-3.4161500930786133,.4,0,0,0)})


def test_reported_scene_has_certified_pose_route_with_original_margins():
    cfg=reported_scene();pose=(-7.771098613739145,-3.4186110496520996,3.1415900000000754)
    scene,route=adaptive_plan(cfg,pose[:2],target=cfg['objects'][0],start_yaw=pose[2],contact=True)
    assert isinstance(route,PoseRoute) and route.start==pose
    assert cfg['robot']['margin']==.25 and cfg['path_following']['tracking_margin']==.1
    assert all(scene.motion_free(m.start,m.v,m.w,m.duration,.1) for m in route.motions)
    assert math.dist(route[-1],cfg['objects'][0]['center'])>1


@pytest.mark.parametrize('values',[{'heading_bins':7},{'max_states':1.5},{'step':0},
                                  {'heuristic_weight':.5},{'reverse_speed':math.nan},{'typo':1}])
def test_pose_settings_reject_invalid_values(values):
    cfg=corridor();cfg['pose_planning']=values
    with pytest.raises(ValueError):PoseSettings.from_config(cfg)


def test_hybrid_honors_shared_budget_and_rejects_invalid_goals():
    cfg=corridor();cfg['planning']={'max_cells':1}
    with pytest.raises(PlanningLimitError,match='grid limit'):
        adaptive_plan(cfg,(3,5),goal=(11,5),start_yaw=0)
    cfg.pop('planning')
    with pytest.raises(ValueError,match='Coordinate goal outside'):
        adaptive_plan(cfg,(3,5),goal=(20,5),start_yaw=0)
    with pytest.raises(ValueError,match='start_yaw'):
        adaptive_plan(cfg,(3,5),goal=(11,5),start_yaw=math.nan)


def test_auto_compares_contact_time_and_keeps_speed_limits():
    cfg=load_config();cfg['objects'][0]['center']=[5,5]
    scene,route=adaptive_plan(cfg,(-4.5,5),target=cfg['objects'][0],start_yaw=0,contact=True)
    assert isinstance(route,PoseRoute)
    assert math.dist(route[-1],(5,5))<1.5
    assert cfg['navigation']['max_linear_speed']==1.4
    assert cfg['navigation']['max_angular_speed']==1.
    assert cfg['robot']['margin']==.25


def test_unplanned_pivot_requires_the_entire_rotation_to_be_free():
    cfg=corridor();scene=FootprintScene(cfg)
    controller=PursuitController(scene,ArcPath([(3,5),(10,5)]),MotionSettings(),FollowerSettings(),scene)
    assert scene.free((3,5,0))
    with pytest.raises(RuntimeError,match='pivot sweep unsafe'):
        controller._rotate((3,5,0),math.pi/2,.03)


def test_pose_heading_deviation_is_checked_against_planned_orientation():
    cfg=corridor();scene=FootprintScene(cfg)
    controller=PursuitController(scene,ArcPath([(3,5),(10,5)]),MotionSettings(),FollowerSettings(),
                                 scene,reference_poses=[(3,5,0),(10,5,0)])
    controller.command((3,5,0),0.)
    with pytest.raises(RuntimeError,match='heading error'):
        controller.command((3,5,.8),.03)


def test_pose_navigation_guard_failure_sends_stop(monkeypatch):
    import sys
    from types import SimpleNamespace as NS
    from navigator import Navigator
    from pose_planner import Motion
    monkeypatch.setitem(sys.modules,'geometry_msgs',NS())
    monkeypatch.setitem(sys.modules,'geometry_msgs.msg',NS(Twist=object))
    monkeypatch.setitem(sys.modules,'rclpy',NS(spin_once=lambda *a,**kw:None))
    cfg=corridor();scene=FootprintScene(cfg)
    route=PoseRoute((3,5,0),[Motion((3,5,0),1,0,5)],5)
    nav=Navigator.__new__(Navigator);nav.grid=scene;nav.node=object();nav.pub=object()
    nav.settings=MotionSettings();nav.follower_settings=FollowerSettings()
    stops=[];monkeypatch.setattr('navigator.stop',lambda pub:stops.append(pub))
    def changed():raise RuntimeError('obstacle moved')
    with pytest.raises(RuntimeError,match='obstacle moved'):nav.follow(route,guard=changed)
    assert stops==[nav.pub]


@pytest.mark.parametrize('limit,values,reason',[
    ('planning',{'max_expansions':1},'search limit'),
    ('pose_planning',{'max_states':1},'state limit')])
def test_pose_search_resource_limits(limit,values,reason):
    cfg=reported_scene();cfg['planner_mode']='hybrid';cfg[limit]=values
    pose=(-7.771098613739145,-3.4186110496520996,math.pi)
    with pytest.raises(PlanningLimitError,match=reason):
        adaptive_plan(cfg,pose[:2],target=cfg['objects'][0],start_yaw=pose[2],contact=True)
