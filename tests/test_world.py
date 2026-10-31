import xml.etree.ElementTree as ET
from generate_world import generate
from grid_map import load_config


def test_world_obstacles_match_config(tmp_path):
    # Minimal installed-source stand-in; real source is checked by simulation.
    source = tmp_path/'source.sdf'
    source.write_text('''<sdf version="1.9"><world name="demo">
    <model name="ground_plane"/><model name="vehicle_green"/>
    <model name="vehicle_blue"><pose>0 2 .325 0 0 0</pose>
    <link name="left_wheel"><pose>.55 .625 0 0 0 0</pose></link>
    <link name="right_wheel"><pose>.55 -.625 0 0 0 0</pose></link>
    <link name="chassis"><pose>-.15 0 0 0 0 0</pose></link>
    <plugin name="gz::sim::systems::DiffDrive"/></model></world></sdf>''')
    output = tmp_path/'maze.sdf'
    cfg = load_config()
    cfg['navigation'] = {'max_linear_speed': .6, 'max_angular_speed': .75}
    generate(source, output, cfg)
    world = ET.parse(output).getroot().find('world')
    assert world.find("model[@name='vehicle_green']") is None
    assert world.find("plugin[@name='gz::sim::systems::Contact']") is not None
    drive = world.find("model[@name='vehicle_blue']/plugin[@name='gz::sim::systems::DiffDrive']")
    assert float(drive.find('max_linear_velocity').text) == .6
    assert float(drive.find('min_linear_velocity').text) == -.6
    assert float(drive.find('max_angular_velocity').text) == .75
    assert float(drive.find('min_angular_velocity').text) == -.75
    for name in ('left_wheel','right_wheel'):
        assert float(world.find(f"model[@name='vehicle_blue']/link[@name='{name}']/pose").text.split()[0]) == 0
    for obj in cfg['walls']+cfg['objects']:
        model = world.find(f"model[@name='{obj['id']}']")
        publisher = model.find("plugin[@name='gz::sim::systems::PosePublisher']")
        if obj in cfg['objects']:
            assert publisher.find('publish_model_pose').text == 'true'
            assert publisher.find('update_frequency').text == '30'
            sensor=model.find("link/sensor[@type='contact']")
            assert sensor.find('contact/collision').text == 'collision'
            assert sensor.find('contact/topic').text == f"/model/{obj['id']}/contact"
        else:
            assert publisher is None
        assert list(map(float,model.find('pose').text.split()[:2])) == obj['center']
        for kind in ['collision','visual']:
            size = model.find(f'link/{kind}/geometry/box/size')
            assert list(map(float,size.text.split())) == obj['size']
