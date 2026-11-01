import math
import sys
from types import SimpleNamespace as NS
import pytest
from grid_map import GridMap
from follower_config import FollowerSettings
from motion_config import MotionSettings
from path_geometry import ArcPath,rounded_path,triangle_free
from pure_pursuit import PursuitController,curvature,integrate,speed_cap,stopping_trajectory_free,preview_speed_cap
from path_planner import plan,route_score


def scene():
    return {'bounds':[0,0,20,20],'resolution':.1,
            'robot':{'radius':.3,'margin':.1},'walls':[],'objects':[]}


def test_curvature_has_correct_robot_frame_sign_and_handles_zero_distance():
    assert curvature((0,0,0),(1,1))==pytest.approx(1)
    assert curvature((0,0,0),(1,-1))==pytest.approx(-1)
    assert curvature((0,0,math.pi/2),(0,2))==pytest.approx(0,abs=1e-9)
    assert curvature((0,0,1),(0,0))==0


def test_corner_rounding_certifies_the_whole_curve_hull():
    grid=GridMap(scene())
    route=[(3,3),(7,3),(7,7)]
    points,stops,count=rounded_path(grid,route)
    assert count==1 and stops==[]
    assert points[0]==route[0] and points[-1]==route[-1]
    assert all(grid.segment_free(a,b) for a,b in zip(points,points[1:]))
    cfg=scene();cfg['walls']=[{'center':[5,5],'size':[.1,.1,1]}]
    assert not triangle_free(GridMap(cfg),[(3,3),(7,3),(5,7)])
    # In a narrow L shaped corner a broad rounding cannot cut the inner wall.
    cfg=scene();cfg['walls']=[{'center':[7,3],'size':[1,1,1]}]
    assert not triangle_free(GridMap(cfg),[(5,3),(7,3),(7,5)])


def test_reverse_corner_remains_a_deliberate_stop():
    points,stops,count=rounded_path(GridMap(scene()),[(3,3),(7,3),(3,3)])
    assert count==0 and len(stops)==1
    assert ArcPath(points,stops).stops==[4.]


def test_arc_length_progress_does_not_skip_a_hairpin_or_move_backwards():
    path=ArcPath([(1,1),(9,1),(9,2),(1,2)])
    station,error=path.project((2,1.9),previous=1)
    assert station<3 and error>.8
    station,_=path.project((1,1),previous=3)
    assert station>=3
    assert path.point(path.length)==(1,2)


def test_velocity_caps_respond_to_curvature_clearance_and_goal():
    motion=MotionSettings(max_linear_speed=1,max_angular_speed=.8);settings=FollowerSettings()
    straight=speed_cap(motion,settings,0,2,10,1)
    assert straight==1
    assert speed_cap(motion,settings,3,2,10,1)<straight
    assert speed_cap(motion,settings,0,.01,10,1)<straight
    assert speed_cap(motion,settings,0,2,.05,.5)==0


def test_braking_tail_includes_motion_after_zero_command():
    cfg=scene();cfg['walls']=[{'center':[4,5],'size':[.1,3,1]}]
    grid=GridMap(cfg)
    assert not stopping_trajectory_free(grid,(3.4,5,0),0,0,1,0,FollowerSettings())
    assert stopping_trajectory_free(grid,(2,5,0),0,0,0,0,FollowerSettings())


def test_upcoming_corner_is_slowed_before_lookahead_reaches_it():
    points,stops,_=rounded_path(GridMap(scene()),[(3,3),(7,3),(7,7)])
    path=ArcPath(points,stops);motion=MotionSettings(max_linear_speed=1,max_angular_speed=.8)
    far=preview_speed_cap(path,0,path.length,1,motion,FollowerSettings())
    near=preview_speed_cap(path,3.2,path.length,1,motion,FollowerSettings())
    assert far==1 and near<.8


@pytest.mark.parametrize('values',[{'lookahead_min':0},{'lookahead_min':2,'lookahead_max':1},
    {'acceleration':2},{'tracking_margin':.01},{'pose_timeout':math.nan},
    {'controller':'unknown'},{'path_objective':'unknown'},{'typo':1},
    {'angular_acceleration':3},{'reaction_time':.1},{'rotate_threshold':2}])
def test_invalid_settings_fail_early(values):
    with pytest.raises(ValueError):FollowerSettings.from_config({'path_following':values})


def test_feedback_time_duplicates_gaps_and_reversal():
    controller=PursuitController(GridMap(scene()),ArcPath([(3,3),(7,3)]),MotionSettings(),FollowerSettings())
    assert controller.command((3,3,0),1)==(0.,0.,False)
    first=controller.command((3,3,0),1.03)
    assert controller.command((3,3,0),1.03)==first
    with pytest.raises(RuntimeError,match='feedback time'):controller.command((3,3,0),1)
    with pytest.raises(RuntimeError,match='feedback time'):controller.command((3,3,0),2)


@pytest.mark.parametrize('course',['curve','obstacle','narrow','initial_heading'])
def test_lagged_kinematics_reach_goal_without_unsafe_shortcuts(course):
    cfg=scene();yaw=0
    if course=='curve':route=[(3,3),(7,3),(7,7)]
    elif course=='obstacle':
        cfg['walls']=[{'center':[8,4],'size':[.5,4,1]}]
        route=[(3,3),(6,7),(11,7)]
    elif course=='narrow':
        cfg['robot']['radius']=.6
        cfg['walls']=[{'center':[1.5,5],'size':[1,8,1]},
                      {'center':[4.5,5],'size':[1,8,1]}]
        route=[(3,2),(3,8)];yaw=math.pi/2
    else:
        route=[(3,3),(8,3)];yaw=math.pi/2
    grid=GridMap(cfg);points,stops,_=rounded_path(grid,route)
    controller=PursuitController(grid,ArcPath(points,stops),
        MotionSettings(max_linear_speed=1,max_angular_speed=.8),FollowerSettings())
    pose=(*route[0],yaw);v=w=0;maximum=0;moving_turns=0
    for step in range(3000):
        cmd_v,cmd_w,done=controller.command(pose,step*.03)
        if done:break
        assert 0<=cmd_v<=1 and abs(cmd_w)<=.8+1e-9
        v+=max(-.03,min(.03,cmd_v-v));w+=max(-.06,min(.06,cmd_w-w))
        nxt=integrate(pose,v,w,.03)
        assert grid.segment_free(pose[:2],nxt[:2])
        pose=nxt;maximum=max(maximum,v)
        moving_turns+=int(v>.05 and abs(w)>.1)
    else:pytest.fail('Continuous follower did not converge')
    assert math.dist(pose[:2],route[-1])<.1
    assert maximum>.7
    if course not in ('narrow','initial_heading'):assert moving_turns>10
    if course=='initial_heading':assert controller.max_error<.10


def test_travel_time_objective_can_replace_a_longer_low_turn_route():
    cfg=scene();grid=GridMap(cfg)
    start,goal=(3,3),(9,8)
    old=plan(grid,start,goal,start_yaw=0,final_yaw=math.pi/2)
    cfg['path_following']={'path_objective':'travel_time'}
    new=plan(GridMap(cfg),start,goal,start_yaw=0,final_yaw=math.pi/2)
    assert route_score(new)[1]<route_score(old)[1]
    assert new==[start,goal]


def test_pursuit_guard_failure_sends_zero(monkeypatch):
    from navigator import Navigator
    monkeypatch.setitem(sys.modules,'rclpy',NS(spin_once=lambda *a,**kw:None))
    monkeypatch.setitem(sys.modules,'geometry_msgs',NS())
    monkeypatch.setitem(sys.modules,'geometry_msgs.msg',NS(Twist=object))
    nav=Navigator.__new__(Navigator);nav.node=object();nav.pub=object()
    nav.grid=GridMap(scene());nav.settings=MotionSettings();nav.follower_settings=FollowerSettings()
    zeros=[];monkeypatch.setattr('navigator.stop',lambda pub:zeros.append(pub))
    def changed():raise RuntimeError('wall moved')
    with pytest.raises(RuntimeError,match='wall moved'):
        nav.follow([(3,3),(7,3)],guard=changed)
    assert zeros==[nav.pub]
