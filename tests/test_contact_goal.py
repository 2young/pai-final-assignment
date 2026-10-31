import math
import subprocess
import sys
import time
from types import SimpleNamespace as NS
import pytest
from contact_goal import footprint_free, plan_contact, ContactMonitor, target_ray_radius, touch, can_resume_touch
from grid_map import GridMap,load_config
from scene_helpers import runtime_config
from pose_provider import WorldPose


def test_only_target_is_excluded_from_contact_clearance():
    cfg=load_config()
    pose=(4.4,4,0)
    assert not GridMap(cfg).point_free(pose[:2])
    assert footprint_free(cfg,pose,'red_box')
    assert not footprint_free(cfg,(0,.5,0),'red_box')
    cfg['objects'].append({'id':'other_box','center':[4.4,4],'size':[.6,.6,.8]})
    assert not footprint_free(cfg,pose,'red_box')


def test_boundary_clearance_uses_orientation_and_rear_overhang():
    cfg=load_config();cfg['bounds']=[-8,-8,8,8]
    cfg=runtime_config(cfg,{'red_box':WorldPose(-1.124,7.069,.4,0,0,0)})
    assert footprint_free(cfg,(-1.135,6.464,math.pi/2),'red_box')
    assert not footprint_free(cfg,(-1.135,6.464,-math.pi/2),'red_box')
    route=plan_contact(GridMap(cfg),(-1.175,4.332),cfg['objects'][0])
    assert route[-1][1] == pytest.approx(4.319)
    assert abs(route[-1][0]+1.175)<.1


def test_rotated_target_can_use_adjacent_free_standoff_cell():
    cfg=runtime_config(load_config(),{'red_box':WorldPose(-1.124,7.069,.4,0,0,math.pi/4)})
    route=plan_contact(GridMap(cfg),(-1.175,4.332),cfg['objects'][0])
    assert GridMap(cfg).planning_view().segment_free(route[-1],route[-1])
    assert route[-1][1]>4.27


def test_unreachable_contact_approach_is_rejected():
    cfg=load_config();cfg['bounds']=[-8,-8,8,8]  # Explicitly sealed field.
    cfg=runtime_config(cfg,{'red_box':WorldPose(1,-3,.4,0,0,0)})
    with pytest.raises(ValueError,match='No reachable approach point'):
        plan_contact(GridMap(cfg),(-5,-4),cfg['objects'][0])


def test_rotated_box_uses_original_dimensions_for_ray_intersection():
    target={'size':[1,1,.8],'body_size':[1,.4,.8],'yaw':math.pi/4}
    assert target_ray_radius(target,math.pi/4) == pytest.approx(.5)
    assert target_ray_radius(target,0) == pytest.approx(.2*math.sqrt(2))


@pytest.mark.parametrize('pose,expected',[
    ((4.4,4,0),True), ((4.4,4,math.pi),False),
    ((2,4,0),False), ((4.8,4,0),False)])
def test_resuming_contact_requires_close_aligned_clear_approach(pose,expected):
    cfg=load_config()
    assert can_resume_touch(cfg,pose,cfg['objects'][0]) == expected


def test_contact_monitor_filters_collision_pair_and_resets():
    monitor=ContactMonitor.__new__(ContactMonitor)
    monitor.target_id='red_box'
    monitor.arm()
    def message(a,b):
        return NS(contacts=[NS(collision1=NS(name=a),collision2=NS(name=b))])
    for a,b in [('red_box::body::collision','ground_plane::link::collision'),
                ('wall::body::collision','vehicle_blue::chassis::collision'),
                ('red_box::body::collision','vehicle_blue_fake::chassis::collision')]:
        monitor.callback(message(a,b))
        assert not monitor.hit
    monitor.callback(message('vehicle_blue::chassis::collision','red_box::body::collision'))
    assert monitor.hit and monitor.details is not None
    monitor.arm()
    assert not monitor.hit and monitor.details is None


@pytest.mark.parametrize('output,ok',[
    ('No publishers on topic\nSubscribers [Address, Message Type]:\nx, gz.msgs.Contacts',False),
    ('Publishers [Address, Message Type]:\nx, gz.msgs.Contacts\nSubscribers []',True)])
def test_sensor_publisher_required_before_motion(monkeypatch,output,ok):
    monitor=ContactMonitor.__new__(ContactMonitor)
    monitor.target_id='red_box'
    monkeypatch.setattr('contact_goal.subprocess.run',lambda *a,**kw:NS(stdout=output))
    if ok:
        monitor.require_sensor()
    else:
        with pytest.raises(RuntimeError,match='No Gazebo contact sensor'):
            monitor.require_sensor()


def test_sensor_discovery_timeout_aborts(monkeypatch):
    monitor=ContactMonitor.__new__(ContactMonitor)
    monitor.target_id='red_box'
    def timeout(*a,**kw):
        raise subprocess.TimeoutExpired('gz',5)
    monkeypatch.setattr('contact_goal.subprocess.run',timeout)
    with pytest.raises(RuntimeError,match='Cannot verify'):
        monitor.require_sensor()


@pytest.mark.parametrize('failure',['moved','travel','timeout'])
def test_contact_failures_stop_and_never_report_success(monkeypatch,capsys,failure):
    monkeypatch.setitem(sys.modules,'rclpy',NS(spin_once=lambda *a,**kw:None))
    monkeypatch.setitem(sys.modules,'geometry_msgs',NS())
    monkeypatch.setitem(sys.modules,'geometry_msgs.msg',NS(Twist=object))
    stopped=[]
    monkeypatch.setattr('contact_goal.stop',lambda pub:stopped.append(pub))
    pose=(4.6,4,0) if failure=='travel' else (2.25,4,0)
    nav=NS(node=object(),pub=object(),provider=NS(current=lambda:pose,stamp=(1,0),received=time.monotonic()))
    monitor=NS(hit=False,arm=lambda *args:None)
    def guard():
        if failure=='moved':
            raise RuntimeError('object moved')
    reason={'moved':'moved','travel':'travel limit','timeout':'timeout'}[failure]
    with pytest.raises(RuntimeError,match=reason):
        touch(nav,load_config(),load_config()['objects'][0],monitor,guard,
              timeout=-1 if failure=='timeout' else 60)
    assert stopped == [nav.pub]
    assert 'CONTACT_REACHED' not in capsys.readouterr().out


def test_queued_old_contact_cannot_confirm_new_approach():
    monitor=ContactMonitor.__new__(ContactMonitor)
    monitor.target_id='red_box'
    monitor.arm((10,0))
    def message(second):
        return NS(header=NS(stamp=NS(sec=second,nanosec=0)),contacts=[
            NS(collision1=NS(name='red_box::body::collision'),
               collision2=NS(name='vehicle_blue::chassis::collision'))])
    monitor.callback(message(9))
    monitor.callback(message(10))
    assert not monitor.hit
    monitor.callback(message(11))
    assert monitor.hit
