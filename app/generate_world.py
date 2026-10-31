"""Reuse the installed Gazebo vehicle, generate all obstacles from maze.json."""
import argparse
import math
from pathlib import Path
import xml.etree.ElementTree as ET
from grid_map import load_config
from motion_config import MotionSettings


def add_pose_publisher(model):
    """Publish model pose periodically, including static models moved by the GUI."""
    plugin = ET.SubElement(model, 'plugin', filename='gz-sim-pose-publisher-system',
                           name='gz::sim::systems::PosePublisher')
    for name, value in [('publish_link_pose','false'), ('publish_sensor_pose','false'),
                        ('publish_collision_pose','false'), ('publish_visual_pose','false'),
                        ('publish_nested_model_pose','false'), ('publish_model_pose','true'),
                        ('use_pose_vector_msg','true'), ('update_frequency','30')]:
        ET.SubElement(plugin, name).text = value


def generate(source, output, config):
    root = ET.parse(source).getroot()
    world = root.find('world')
    world.set('name', 'maze')
    ET.SubElement(world, 'plugin', filename='gz-sim-contact-system',
                  name='gz::sim::systems::Contact')
    for model in list(world.findall('model')):
        if model.get('name') not in ('vehicle_blue', 'ground_plane'):
            world.remove(model)
    vehicle = world.find("model[@name='vehicle_blue']")
    if vehicle is None:
        raise ValueError('Installed diff_drive.sdf has no vehicle_blue')
    # DiffDrive controls the axle midpoint, not the original model origin.
    # Translate all links together so model/world pose is the control point.
    wheel_links = [vehicle.find(f"link[@name='{name}']")
                   for name in ('left_wheel', 'right_wheel')]
    axle_x = sum(float(link.find('pose').text.split()[0]) for link in wheel_links)/2
    for link in vehicle.findall('link'):
        element = link.find('pose')
        values = list(map(float, element.text.split()))
        values[0] -= axle_x
        element.text = ' '.join(map(str, values))
    envelope = 0.0
    for link in vehicle.findall('link'):
        values = list(map(float, link.find('pose').text.split()))
        for collision in link.findall('collision'):
            if collision.find('pose') is not None:
                raise ValueError('Unexpected collision offset; remeasure footprint')
            geometry = collision.find('geometry')
            sphere = geometry.find('sphere/radius')
            box = geometry.find('box/size')
            if sphere is not None:
                bound = math.hypot(*values[:2]) + float(sphere.text)
                hx = hy = float(sphere.text)
            elif box is not None and all(abs(v) < 1e-6 for v in values[3:]):
                sx, sy, _ = map(float, box.text.split())
                bound = math.hypot(abs(values[0])+sx/2, abs(values[1])+sy/2)
                hx,hy = sx/2,sy/2
            else:
                raise ValueError('Unexpected vehicle geometry; remeasure footprint')
            envelope = max(envelope, bound)
            if 'footprint_bounds' in config['robot']:
                x0,y0,x1,y1 = config['robot']['footprint_bounds']
                if not (x0 <= values[0]-hx and values[0]+hx <= x1 and
                        y0 <= values[1]-hy and values[1]+hy <= y1):
                    raise ValueError('Robot footprint_bounds does not contain collision geometry')
    if config['robot']['radius'] < envelope:
        raise ValueError(f"Robot radius too small: need at least {envelope:.3f}m")
    x, y, yaw = config['robot']['spawn']
    pose = vehicle.find('pose').text.split()
    vehicle.find('pose').text = f'{x} {y} {pose[2]} 0 0 {yaw}'
    drive = vehicle.find("plugin[@name='gz::sim::systems::DiffDrive']")
    settings = MotionSettings.from_config(config)
    for name, value in [('odom_publish_frequency', '30'),
                        ('max_linear_velocity', str(settings.max_linear_speed)),
                        ('min_linear_velocity', str(-settings.max_linear_speed)),
                        ('max_angular_velocity', str(settings.max_angular_speed)),
                        ('min_angular_velocity', str(-settings.max_angular_speed))]:
        el = drive.find(name)
        if el is None:
            el = ET.SubElement(drive, name)
        el.text = value
    # Ground truth world pose, independent of wheel slip and odometry origin.
    add_pose_publisher(vehicle)
    for obj in config['walls'] + config['objects']:
        model = ET.SubElement(world, 'model', name=obj['id'])
        ET.SubElement(model, 'static').text = 'true'
        x, y = obj['center']
        sx, sy, sz = obj['size']
        ET.SubElement(model, 'pose').text = f'{x} {y} {sz/2} 0 0 0'
        if obj in config['objects']:
            add_pose_publisher(model)
        link = ET.SubElement(model, 'link', name='body')
        if obj in config['objects']:
            sensor = ET.SubElement(link, 'sensor', name='contact_sensor', type='contact')
            ET.SubElement(sensor, 'always_on').text = 'true'
            ET.SubElement(sensor, 'update_rate').text = '30'
            contact = ET.SubElement(sensor, 'contact')
            ET.SubElement(contact, 'collision').text = 'collision'
            ET.SubElement(contact, 'topic').text = f"/model/{obj['id']}/contact"
        for tag in ('collision','visual'):
            element = ET.SubElement(link, tag, name=tag)
            box = ET.SubElement(ET.SubElement(element, 'geometry'), 'box')
            ET.SubElement(box, 'size').text = f'{sx} {sy} {sz}'
            if tag == 'visual':
                material = ET.SubElement(element, 'material')
                color = ' '.join(map(str,obj.get('color',[0.6,0.6,0.6,1])))
                for field in ('ambient','diffuse'):
                    ET.SubElement(material, field).text = color
    ET.indent(root)
    Path(output).parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(output, encoding='unicode', xml_declaration=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--source', required=True)
    parser.add_argument('--output', default='/workspace/worlds/maze.sdf')
    args = parser.parse_args()
    generate(args.source, args.output, load_config())
