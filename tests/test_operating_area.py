import math
from types import SimpleNamespace as NS
import sys
import pytest
from grid_map import GridMap,load_config
from object_tracker import runtime_config
from pose_provider import WorldPose
from adaptive_planner import adaptive_plan,planning_windows
from planning_limits import PlanningSettings,PlanningLimitError
from contact_goal import footprint_free
from boundary_entry import plan_boundary_entry,execute_boundary_entry


def live(x,y=4,yaw=0):
    return runtime_config(load_config(),{'red_box':WorldPose(x,y,.4,0,0,yaw)})


def test_old_field_outside_position_now_accepted_without_clamping():
    cfg=load_config();observed=runtime_config(cfg,{'red_box':WorldPose(8.4,4,.4,0,0,0)})
    assert observed['objects'][0]['center']==[8.4,4]
    assert cfg['objects'][0]['center']==[5,4]
    grid,route=adaptive_plan(observed,(4.4,4),target=observed['objects'][0],start_yaw=0,contact=True)
    assert all(grid.segment_free(a,b) for a,b in zip(route,route[1:]))
    assert grid.bounds!=observed['bounds']


def test_straddling_object_keeps_real_pose_and_has_reachable_inner_face():
    cfg=live(12)
    target=cfg['objects'][0]
    assert target['crosses_boundary'] and target['center']==[12,4]
    grid,route=adaptive_plan(cfg,(9.25,4),target=target,start_yaw=0,contact=True)
    assert route[-1][0]<12
    assert footprint_free(cfg,(11.4,4,0),'red_box')
    assert not footprint_free(cfg,(11.6,4,0),'red_box')


def test_partial_overlap_does_not_guarantee_safe_contact():
    cfg=live(12.2)
    assert cfg['objects'][0]['crosses_boundary']
    with pytest.raises(ValueError,match='No reachable approach/route'):
        adaptive_plan(cfg,(9.45,4),target=cfg['objects'][0],start_yaw=0,contact=True)


def test_wholly_outside_and_aabb_only_corner_overlap_are_rejected():
    with pytest.raises(ValueError,match='outside operating area.*center='):
        live(13)
    cfg=load_config();cfg['objects'][0]['size']=[2,.1,.8]
    with pytest.raises(ValueError,match='outside operating area'):
        runtime_config(cfg,{'red_box':WorldPose(12.3,12.3,.4,0,0,-math.pi/4)})


def test_legacy_whole_object_policy_remains_available():
    cfg=load_config();cfg['boundary_policy']='whole_object'
    with pytest.raises(ValueError,match='outside operating area'):
        runtime_config(cfg,{'red_box':WorldPose(12,4,.4,0,0,0)})


def small_scene():
    return {'bounds':[-12,-12,12,12],'resolution':.5,
            'robot':{'radius':.2,'margin':.1},'objects':[],
            'walls':[{'id':'barrier','center':[0,0],'size':[.3,12,1]}],
            'planning':{'initial_padding':1.,'expansion_step':2.,'max_attempts':4}}


def test_search_expands_to_find_route_around_long_wall(capsys):
    cfg=small_scene()
    grid,route=adaptive_plan(cfg,(-3,0),goal=(3,0))
    assert grid.bounds==cfg['bounds']
    assert max(abs(y) for x,y in route)>6
    assert 'Planning window 4:' in capsys.readouterr().out
    assert all(grid.segment_free(a,b) for a,b in zip(route,route[1:]))


def test_window_edge_is_not_an_inflated_operating_wall():
    cfg=small_scene();cfg['walls']=[]
    grid=GridMap(cfg,bounds=[-4,-1,4,1])
    assert grid.point_free((-3.75,.25))
    assert grid.operating_bounds==[-12,-12,12,12]
    assert grid.planning_view().bounds==[-4,-1,4,1]
    assert grid.cell_center(grid.cell((-3.75,.25)))==(-3.75,.25)


def test_float_snapped_window_keeps_full_number_of_cells():
    cfg=load_config()
    grid=GridMap(cfg,bounds=[6.1000000000000005,1,12,7])
    assert grid.width==59 and grid.height==60


@pytest.mark.parametrize('setting,value,reason',[
    ('max_cells',10,'grid limit'),('max_expansions',1,'search limit'),
    ('max_attempts',1,'window attempt limit')])
def test_planning_limits_fail_with_distinct_reasons(setting,value,reason):
    cfg=small_scene();cfg['planning'][setting]=value
    with pytest.raises(PlanningLimitError,match=reason):
        adaptive_plan(cfg,(-3,0),goal=(3,0))


def test_time_budget_is_enforced(monkeypatch):
    values=iter([0,2])
    monkeypatch.setattr('planning_limits.time.monotonic',lambda:next(values,2))
    cfg=small_scene();cfg['planning']['timeout']=1
    with pytest.raises(PlanningLimitError,match='time limit'):
        adaptive_plan(cfg,(-3,0),goal=(3,0))


def test_fully_sealed_operating_area_is_not_reported_as_budget_failure():
    cfg=small_scene();cfg['walls'][0]['size'][1]=24
    with pytest.raises(ValueError,match='No reachable approach/route within operating area'):
        adaptive_plan(cfg,(-3,0),goal=(3,0))


def test_outside_coordinate_goal_is_rejected():
    with pytest.raises(ValueError,match='Coordinate goal outside operating area'):
        adaptive_plan(small_scene(),(-3,0),goal=(20,0))


@pytest.mark.parametrize('values',[{'max_cells':0},{'max_attempts':1.5},{'timeout':math.nan},
                                    {'initial_padding':-1},{'unknown':2}])
def test_invalid_planning_settings_are_rejected(values):
    with pytest.raises(ValueError):PlanningSettings.from_config({'planning':values})


def test_boundary_entry_can_back_out_after_contact_then_replan():
    cfg=live(5)
    entry=plan_boundary_entry(cfg,(11.4,4,0))
    assert entry['sign']==-1 and entry['goal'][0]<9.7
    grid,route=adaptive_plan(cfg,entry['goal'],target=cfg['objects'][0],start_yaw=0,contact=True)
    assert grid.planning_view().segment_free(route[0],route[0])
    assert plan_boundary_entry(cfg,(0,5,0)) is None


def test_boundary_entry_refuses_blocking_objects_or_tangent_heading():
    with pytest.raises(ValueError,match='No safe straight entry'):
        plan_boundary_entry(live(5),(10.7,4,math.pi/2))
    with pytest.raises(ValueError,match='No safe straight entry'):
        plan_boundary_entry(live(8.38),(11.35,4,0))


def test_boundary_entry_guard_failure_sends_stop(monkeypatch):
    monkeypatch.setitem(sys.modules,'rclpy',NS(spin_once=lambda *a,**kw:None))
    monkeypatch.setitem(sys.modules,'geometry_msgs',NS())
    monkeypatch.setitem(sys.modules,'geometry_msgs.msg',NS(Twist=object))
    stops=[];monkeypatch.setattr('boundary_entry.stop',lambda pub:stops.append(pub))
    nav=NS(node=object(),pub=object())
    entry={'start':(11.4,4),'goal':(9.65,4),'heading':0,'sign':-1}
    def fail():raise RuntimeError('object moved')
    with pytest.raises(RuntimeError,match='moved'):
        execute_boundary_entry(nav,live(5),entry,fail)
    assert stops==[nav.pub]
