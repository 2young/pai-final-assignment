import itertools
import math
import random
import pytest
from grid_map import GridMap
from path_planner import astar,plan,route_score,simplify_path
from route_metrics import MotionMetrics


def cells_grid(blocked=(),size=6):
    grid=GridMap.__new__(GridMap)
    grid.bounds=[0,0,size,size]
    grid.width=grid.height=size
    grid.resolution=1
    grid.blocked=set(blocked)
    return grid


def open_config():
    return {'bounds':[0,0,10,10],'resolution':.1,
            'robot':{'radius':.3,'margin':.1},'walls':[],'objects':[]}


def test_eight_neighbor_astar_uses_diagonal_distance():
    grid=cells_grid()
    path=astar(grid,(1,1),(4,3))
    assert len(path)==4
    assert sum(math.dist(a,b) for a,b in zip(path,path[1:]))==pytest.approx(1+2*math.sqrt(2))


def test_diagonal_cannot_squeeze_between_blocked_neighbors():
    grid=cells_grid({(1,0)},size=2)
    assert astar(grid,(0,0),(1,1))==[(0,0),(0,1),(1,1)]
    grid.blocked.add((0,1))
    with pytest.raises(ValueError,match='No route'):
        astar(grid,(0,0),(1,1))


@pytest.mark.parametrize('a,b,blocked',[
    ((.5,.5),(3.5,3.5),(2,1)),
    ((.5,1),(4.5,1),(2,0)),
    ((.99,1.01),(1.01,.99),(1,1)),
    ((1,.5),(1,4.5),(0,2))])
def test_supercover_rejects_even_corner_and_edge_contacts(a,b,blocked):
    grid=cells_grid({blocked})
    assert not grid.segment_free(a,b)
    assert not grid.segment_free(b,a)


def test_supercover_matches_independent_closed_box_intersections():
    grid=cells_grid()
    def intersects(a,b,i,j):
        low,high=0.,1.
        for p,q,edge in zip(a,b,(i,j)):
            delta=q-p
            if abs(delta)<1e-14:
                if not edge<=p<=edge+1:
                    return False
            else:
                t0,t1=sorted(((edge-p)/delta,(edge+1-p)/delta))
                low,high=max(low,t0),min(high,t1)
                if low>high+1e-12:
                    return False
        return True
    rng=random.Random(23)
    for _ in range(100):
        a=tuple(rng.uniform(.1,5.9) for _ in range(2))
        b=tuple(rng.uniform(.1,5.9) for _ in range(2))
        expected={(i,j) for i in range(6) for j in range(6) if intersects(a,b,i,j)}
        assert grid.segment_cells(a,b)==expected
        assert grid.segment_cells(b,a)==expected


def test_open_space_route_is_one_arbitrary_angle_segment():
    grid=GridMap(open_config())
    start,goal=(1.2,1.2),(7.8,5.3)
    route=plan(grid,start,goal)
    assert route==[start,goal]
    assert route_score(route)[0]==0
    assert grid.planning_view().segment_free(*route)


def test_initial_and_final_heading_can_prefer_one_turn_over_shorter_diagonal():
    grid=GridMap(open_config())
    start,goal=(1.2,1.2),(7.8,5.3)
    route=plan(grid,start,goal,start_yaw=0,final_yaw=math.pi/2)
    score=route_score(route,0,math.pi/2)
    assert score[0]==1
    assert route_score([start,goal],0,math.pi/2)[0]==2
    assert score[1]>math.dist(start,goal)


def test_visibility_search_matches_exhaustive_candidate_routes(monkeypatch):
    grid=cells_grid({(2,2),(2,3)})
    nodes=[(1.5,2.5),(4.5,2.5),(1.5,.5),(4.5,.5),(1.5,4.5),(4.5,4.5)]
    monkeypatch.setattr('path_planner._candidates',lambda *a:nodes)
    scores=[]
    for count in range(5):
        for middle in itertools.permutations(nodes[2:],count):
            route=[nodes[0],*middle,nodes[1]]
            if all(grid.segment_free(a,b) for a,b in zip(route,route[1:])):
                scores.append(route_score(route,math.pi/2,0))
    actual=simplify_path(grid,[nodes[0],nodes[1]],math.pi/2,0)
    assert route_score(actual,math.pi/2,0)==pytest.approx(min(scores))


def test_motion_metrics_count_episodes_and_wrap_yaw():
    metrics=MotionMetrics()
    metrics.observe_pose((0,0,math.radians(179)))
    metrics.observe_pose((0,0,math.radians(-179)))
    metrics.observe_command(0,.5)
    metrics.observe_command(0,.4)
    metrics.observe_command(.2,.1)
    metrics.observe_command(0,-.5)
    metrics.stopped()
    assert metrics.summary()['rotation_episodes']==2
    assert metrics.summary()['total_yaw_degrees']==pytest.approx(2)


def test_non_axis_initial_and_terminal_heading_join():
    grid=GridMap(open_config())
    start,goal=(1.2,1.2),(5.3,5.7)
    initial,final=math.radians(30),math.radians(100)
    route=plan(grid,start,goal,start_yaw=initial,final_yaw=final)
    assert route_score(route,initial,final)[0]==1
    assert math.atan2(route[1][1]-start[1],route[1][0]-start[0])==pytest.approx(initial)
    assert math.atan2(goal[1]-route[-2][1],goal[0]-route[-2][0])==pytest.approx(final)


@pytest.mark.parametrize('heading',[math.nan,math.inf])
def test_nonfinite_heading_is_rejected(heading):
    with pytest.raises(ValueError,match='Non-finite planning heading'):
        plan(GridMap(open_config()),(1,1),(5,5),start_yaw=heading)


def test_far_outside_segment_is_rejected_before_grid_traversal():
    assert not cells_grid().segment_free((.5,.5),(1e12,1e12))
