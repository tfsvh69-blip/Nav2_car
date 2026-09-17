"""整车 Xacro 的离线结构回归检查。"""

from pathlib import Path
import struct
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import pytest
import xacro


EXPECTED_LINKS = {
    'base_footprint',
    'base_link',
    'upper_body_link',
    'compute_link',
    'front_left_wheel_link',
    'front_right_wheel_link',
    'rear_left_wheel_link',
    'rear_right_wheel_link',
    'imu_link',
    'laser_frame',
    'camera_link',
    'camera_mount_link',
}


def _expanded_robot():
    share = Path(get_package_share_directory('carcar_description'))
    document = xacro.process_file(str(share / 'urdf' / 'carcar.urdf.xacro'))
    return ET.fromstring(document.toxml())


def test_xml_description_launch_publishes_xacro_model_once():
    share = Path(get_package_share_directory('carcar_description'))
    launch = ET.parse(share / 'launch' / 'description.launch.xml').getroot()
    publishers = [
        node for node in launch.findall('node')
        if node.attrib.get('pkg') == 'robot_state_publisher'
    ]
    assert len(publishers) == 1
    params = {
        param.attrib['name']: param.attrib.get('value')
        for param in publishers[0].findall('param')
    }
    assert params['robot_description'] == "$(command 'xacro $(var model)')"
    assert params['use_sim_time'] == '$(var use_sim_time)'


def _xyz(joint):
    values = joint.find('origin').attrib['xyz'].split()
    return tuple(float(value) for value in values)


def _rpy(joint):
    values = joint.find('origin').attrib['rpy'].split()
    return tuple(float(value) for value in values)


def test_model_has_one_connected_tree_and_expected_frames():
    robot = _expanded_robot()
    links = {link.attrib['name'] for link in robot.findall('link')}
    joints = robot.findall('joint')

    assert links == EXPECTED_LINKS
    assert len({joint.attrib['name'] for joint in joints}) == len(joints)

    child_names = [joint.find('child').attrib['link'] for joint in joints]
    assert len(set(child_names)) == len(child_names)
    assert set(child_names) == links - {'base_footprint'}
    parent_names = [joint.find('parent').attrib['link'] for joint in joints]
    assert all(parent in links for parent in parent_names)


def test_confirmed_wheel_geometry_and_base_height_do_not_regress():
    robot = _expanded_robot()
    joints = {joint.attrib['name']: joint for joint in robot.findall('joint')}

    assert _xyz(joints['base_footprint_joint']) == (0.0, 0.0, 0.03)
    assert _xyz(joints['front_left_wheel_joint']) == (0.06, 0.0925, 0.0)
    assert _xyz(joints['front_right_wheel_joint']) == (0.06, -0.0925, 0.0)
    assert _xyz(joints['rear_left_wheel_joint']) == (-0.06, 0.0925, 0.0)
    assert _xyz(joints['rear_right_wheel_joint']) == (-0.06, -0.0925, 0.0)

    path = "./link[@name='front_left_wheel_link']/collision/geometry/cylinder"
    wheel = robot.find(path)
    assert float(wheel.attrib['radius']) == 0.03
    assert float(wheel.attrib['length']) == 0.031


def test_sensor_ownership_and_collision_geometry():
    robot = _expanded_robot()
    links = {link.attrib['name']: link for link in robot.findall('link')}

    # RealSense 内部 optical frame 必须由官方驱动发布，描述包不能重复定义。
    assert not any(name.endswith('optical_frame') for name in links)
    for name in EXPECTED_LINKS - {'base_footprint'}:
        assert links[name].find('collision') is not None


def test_confirmed_body_sensor_transforms_and_exact_meshes():
    robot = _expanded_robot()
    joints = {joint.attrib['name']: joint for joint in robot.findall('joint')}

    # 用户给出的离地高度先减去 30 mm 的 base_link 离地高度。
    assert _xyz(joints['upper_body_joint']) == pytest.approx((0.0, 0.0, 0.105))
    assert _xyz(joints['laser_joint']) == pytest.approx((-0.05, 0.0, 0.187))
    assert _xyz(joints['camera_joint']) == pytest.approx((0.13, 0.033, 0.14))
    assert _rpy(joints['camera_joint']) == pytest.approx((3.141592653589793, 0.0, 0.0))
    assert _xyz(joints['camera_mount_joint']) == pytest.approx((0.13, 0.033, 0.14))
    assert _rpy(joints['camera_mount_joint']) == (0.0, 0.0, 0.0)

    expected_meshes = {
        'carcar_upper_structure.stl',
        'lidar_mount_plate.stl',
        'camera_lower_mount.stl',
        'camera_direct_mount.stl',
        'jetson_lower_mount.stl',
    }
    meshes = robot.findall('.//visual/geometry/mesh')
    vehicle_meshes = {
        Path(mesh.attrib['filename']).name: mesh for mesh in meshes
        if '/meshes/vehicle/' in mesh.attrib['filename']
    }
    assert set(vehicle_meshes) == expected_meshes
    assert all(mesh.attrib['scale'] == '0.001 0.001 0.001'
               for mesh in vehicle_meshes.values())


def test_user_meshes_are_installed_binary_stl_and_deck_is_horizontal():
    share = Path(get_package_share_directory('carcar_description'))
    robot = _expanded_robot()
    for mesh in robot.findall('.//visual/geometry/mesh'):
        relative = mesh.attrib['filename'].removeprefix('package://carcar_description/')
        data = (share / relative).read_bytes()
        count = struct.unpack_from('<I', data, 80)[0]
        assert count > 0
        assert len(data) == 84 + 50 * count

    # STL 顶板大平面为原 Y=130 mm，安装后必须朝上，不得误用单位旋转。
    visual = robot.find("./link[@name='upper_body_link']/visual[@name='upper_structure_mesh']")
    rpy = tuple(float(v) for v in visual.find('origin').attrib['rpy'].split())
    assert rpy == pytest.approx((1.5707963267948966, 0.0, 0.0))
    z = float(visual.find('origin').attrib['xyz'].split()[2])
    assert 0.03 + 0.105 + z + 0.130 == pytest.approx(0.140)
