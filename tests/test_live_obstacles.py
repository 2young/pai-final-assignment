import math
import sys
from types import SimpleNamespace as NS
import pytest
from object_tracker import ObjectTracker,runtime_config,scene_models
from pose_provider import WorldPose
from grid_map import GridMap
from path_planner import plan
from contact_goal import footprint_free
from boundary_entry import center_clear
from ros_feedback import spin_some


def config():
    return {'bounds':[-12,-12,12,12],'resolution':.5,
            'robot':{'radius':.2,'margin':.1,'footprint_bounds':[-.2,-.2,.2,.2]},
            'walls':[{'id':'gate','center':[0,0],'size':[.4,24,1]},
                     {'id':'block','center':[10,10],'size':[1,1,1]}],
            'objects':[{'id':'red_box','center':[5,5],'size':[.6,.6,.8]}]}


def observations(cfg):
    return {o['id']:WorldPose(*o['center'],o['size'][2]/2,0,0,0) for o in scene_models(cfg)}


def test_missing_obstacle_never_falls_back_to_json():
    cfg=config();poses=observations(cfg);del poses['gate']
    with pytest.raises(RuntimeError,match='gate.*all scene models required'):
        runtime_config(cfg,poses)


def test_moving_gate_opens_old_occupancy_and_closes_new_position():
    cfg=config();poses=observations(cfg)
    with pytest.raises(ValueError,match='No route'):
        plan(GridMap(runtime_config(cfg,poses)),(-3,0),(3,0))
    poses['gate']=WorldPose(20,0,.5,0,0,0)
    live=runtime_config(cfg,poses);grid=GridMap(live)
    assert cfg['walls'][0]['center']==[0,0]
    assert live['walls'][0]['center']==[20,0]
    assert grid.point_free((0,0))
    assert plan(grid,(-3,0),(3,0))==[(-3,0),(3,0)]
    poses['gate']=WorldPose(-3,0,.5,0,0,0)
    blocked=runtime_config(cfg,poses)
    assert not GridMap(blocked).point_free((-3,0))
    assert not footprint_free(blocked,(-3,0,0),None)
    assert not center_clear(blocked,(-3,0))


def test_rotated_wall_uses_original_dimensions_and_world_aabb():
    cfg=config();cfg['walls'][0]['size']=[.4,4,1]
    poses=observations(cfg);poses['gate']=WorldPose(1,2,.5,0,0,math.pi/2)
    live=runtime_config(cfg,poses);wall=live['walls'][0]
    assert wall['center']==[1,2]
    assert wall['body_size']==[.4,4,1]
    assert wall['size']==pytest.approx([4,.4,1])
    assert not footprint_free(live,(2,2,0),'red_box')
    assert footprint_free(live,(1,4,0),'red_box')


def test_wall_junctions_allowed_but_target_overlap_uses_observed_wall_pose():
    cfg=config();poses=observations(cfg)
    poses['block']=WorldPose(0,0,.5,0,0,0)
    runtime_config(cfg,poses)  # Connected walls are legitimate.
    poses['block']=WorldPose(5,5,.5,0,0,0)
    with pytest.raises(ValueError,match='red_box: overlaps block'):
        runtime_config(cfg,poses)


@pytest.mark.parametrize('state',[WorldPose(0,0,1,0,0,0),WorldPose(0,0,.5,.1,0,0),
                                   WorldPose(math.nan,0,.5,0,0,0)])
def test_invalid_obstacle_pose_is_rejected(state):
    cfg=config();poses=observations(cfg);poses['gate']=state
    with pytest.raises(ValueError,match='gate:'):
        runtime_config(cfg,poses)


def test_tracker_subscribes_to_every_registered_wall_and_object(monkeypatch):
    monkeypatch.setattr('object_tracker.PoseProvider',lambda node,name:name)
    tracker=ObjectTracker(object(),config())
    assert set(tracker.providers)=={'gate','block','red_box'}


def test_each_snapshot_requires_new_observations_for_all_models(monkeypatch):
    cfg=config();clock=[100.];poses=observations(cfg)
    monkeypatch.setattr('object_tracker.time.monotonic',lambda:clock[0])
    monkeypatch.setattr('object_tracker.PoseProvider',lambda node,name:NS(
        state=None,received=0,current_state=lambda:poses[name]))
    tracker=ObjectTracker(object(),cfg)
    calls=[]
    def receive(*a,**kw):
        calls.append(True)
        for name,p in tracker.providers.items():p.state=poses[name];p.received=clock[0]
    monkeypatch.setattr('object_tracker.spin_some',receive)
    tracker.snapshot()
    clock[0]=200.;poses['gate']=WorldPose(1,0,.5,0,0,0)
    live=tracker.snapshot()
    assert len(calls)==2 and live['walls'][0]['center']==[1,0]
    assert tracker.baseline['gate']==poses['gate']


def test_snapshot_missing_wall_names_it_and_rejects_incoherent_scene(monkeypatch):
    cfg=config();poses=observations(cfg)
    monkeypatch.setattr('object_tracker.time.monotonic',lambda:100)
    monkeypatch.setattr('object_tracker.spin_some',lambda *a,**kw:None)
    monkeypatch.setattr('object_tracker.PoseProvider',lambda node,name:NS(
        state=poses[name],received=100,current_state=lambda:poses[name]))
    tracker=ObjectTracker(object(),cfg);tracker.providers['gate'].received=99
    with pytest.raises(RuntimeError,match="No fresh object world pose: .*gate"):
        tracker.snapshot(timeout=0)
    tracker.providers['gate'].received=100.2
    with pytest.raises(RuntimeError,match='not time-coherent'):
        tracker.snapshot(timeout=0)


@pytest.mark.parametrize('moved',[WorldPose(.06,0,.5,0,0,0),WorldPose(0,0,.5,0,0,.06)])
def test_wall_movement_guard_also_stops_navigation(monkeypatch,moved):
    from navigator import Navigator
    monkeypatch.setitem(sys.modules,'rclpy',NS(spin_once=lambda *a,**kw:None))
    monkeypatch.setitem(sys.modules,'geometry_msgs',NS())
    monkeypatch.setitem(sys.modules,'geometry_msgs.msg',NS(Twist=object))
    tracker=ObjectTracker.__new__(ObjectTracker)
    tracker.baseline={'gate':WorldPose(0,0,.5,0,0,0)}
    tracker.providers={'gate':NS(current_state=lambda:moved)}
    stopped=[];monkeypatch.setattr('navigator.stop',lambda pub:stopped.append(pub))
    nav=Navigator.__new__(Navigator);nav.node=object();nav.pub=object()
    with pytest.raises(RuntimeError,match='gate: object moved'):
        nav.follow([(0,0),(1,0)],guard=tracker.assert_unchanged)
    assert stopped==[nav.pub]


def test_feedback_batch_is_bounded_and_processes_multiple_callbacks(monkeypatch):
    calls=[]
    monkeypatch.setitem(sys.modules,'rclpy',NS(spin_once=lambda *a,**kw:calls.append(kw['timeout_sec'])))
    spin_some(object(),timeout_sec=.02,max_callbacks=12,budget=1)
    assert calls==[.02]+[0.]*11


def test_duplicate_scene_ids_are_rejected():
    cfg=config();cfg['walls'][0]['id']='red_box'
    with pytest.raises(ValueError,match='unique'):
        scene_models(cfg)
