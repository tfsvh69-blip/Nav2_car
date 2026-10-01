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
    'camera_tilt_link',
    'camera_servo_left_foot_link',
    'camera_servo_right_foot_link',
}


def _expanded_robot(mappings=None):
    share = Path(get_package_share_directory('carcar_description'))
    document = xacro.process_file(
        str(share / 'urdf' / 'carcar.urdf.xacro'), mappings=mappings)
    return ET.fromstring(document.toxml())


def test_xml_description_launch_publishes_xacro_model_once():
    share = Path(get_package_share_directory('carcar_description'))
    launch = ET.parse(share / 'launch' / 'description.launch.xml').getroot()
    publishers = [
        node for node in launch.findall('node')
        if node.attrib.get('pkg') == 'robot_state_publisher'
    ]
    assert len(publishers) == 1
    assert publishers[0].attrib['if'] == '$(var publish_model)'
    assert launch.find("./arg[@name='publish_model']").attrib['default'] == 'true'
    assert launch.find("./arg[@name='use_rviz']").attrib['default'] == 'false'
    assert launch.find("./arg[@name='preview_wheels']").attrib['default'] == 'false'
    rviz = launch.find("./node[@pkg='rviz2']")
    assert rviz.attrib['if'] == '$(var use_rviz)'
    assert rviz.attrib['args'] == '-d $(find-pkg-share carcar_description)/rviz/carcar_model.rviz'
    preview = launch.find("./node[@pkg='joint_state_publisher']")
    assert preview.attrib['if'] == '$(var preview_wheels)'
    params = {
        param.attrib['name']: param.attrib.get('value')
        for param in publishers[0].findall('param')
    }
    assert params['robot_description'] == (
        "$(command 'xacro $(var model) "
        "camera_extrinsics_file:=$(var camera_extrinsics_file) "
        "camera_tilt_dynamic:=$(var camera_tilt_dynamic)')"
    )
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


def test_servo_feet_bolt_to_upper_deck():
    robot = _expanded_robot()
    rear_joint = robot.find("./joint[@name='camera_servo_left_foot_joint']")
    front_joint = robot.find("./joint[@name='camera_servo_right_foot_joint']")
    rear = robot.find("./link[@name='camera_servo_left_foot_link']/visual[@name='camera_servo_left_foot']")
    front = robot.find("./link[@name='camera_servo_right_foot_link']/visual[@name='camera_servo_right_foot']")
    pivot_height = 0.145
    deck_top = 0.135

    # STL 的原始 Y=0 面为带双孔的平底；两件都应贴住顶板。
    assert pivot_height + _xyz(rear_joint)[2] == pytest.approx(deck_top)
    assert pivot_height + _xyz(front_joint)[2] == pytest.approx(deck_top)
    assert _rpy(rear_joint) == pytest.approx((1.57079632679, 0.0, 0.0))
    assert _rpy(front_joint) == pytest.approx((1.57079632679, 0.0, 0.0))

    # 左右 STL 的原始 X 宽 10 mm，原始 Z 长 20 mm；变换后前后各贴舵机端面，
    # 两孔沿车体 Y 排列，前端螺钉中心仍在顶板 X=110 mm 前边界内。
    assert 0.090 + _xyz(rear_joint)[0] + 0.010 == pytest.approx(0.062)
    assert 0.090 + _xyz(front_joint)[0] == pytest.approx(0.102)
    assert 0.090 + _xyz(front_joint)[0] + 0.0055 < 0.110
    assert _xyz(front_joint)[1] == pytest.approx(_xyz(rear_joint)[1])
    assert 0.090 + _xyz(front_joint)[0] + 0.005 == pytest.approx(0.107)
    assert _xyz(front_joint)[1] - 0.045392723 == pytest.approx(-0.030)
    assert rear.find('material').attrib['name'] == 'mount_orange'
    assert front.find('material').attrib['name'] == 'mount_orange'


def test_confirmed_body_sensor_transforms_and_exact_meshes():
    robot = _expanded_robot()
    joints = {joint.attrib['name']: joint for joint in robot.findall('joint')}

    # 用户给出的离地高度先减去 30 mm 的 base_link 离地高度。
    assert _xyz(joints['upper_body_joint']) == pytest.approx((0.0, 0.0, 0.100))
    assert _xyz(joints['laser_joint']) == pytest.approx((-0.05, 0.0, 0.187))
    assert _xyz(joints['camera_mount_joint']) == pytest.approx((0.09, 0.0, 0.115))
    assert joints['camera_tilt_joint'].attrib['type'] == 'fixed'
    assert _xyz(joints['camera_joint']) == pytest.approx((0.06, 0.0175, 0.010))
    assert _rpy(joints['camera_joint']) == pytest.approx((0.0, 0.0, 0.0))
    assert _rpy(joints['camera_mount_joint']) == (0.0, 0.0, 0.0)

    bracket = robot.find("./link[@name='camera_tilt_link']/visual[@name='camera_tilt_bracket']")
    bracket_origin = _xyz(bracket)
    assert _rpy(bracket) == pytest.approx((0.0, 0.0, 0.0))
    # STL 小端轴孔约 (9,35,8.5) mm，应落在舵机转轴；大端约 X=38 mm 朝向相机。
    assert tuple(a + b for a, b in zip(bracket_origin, (0.009, 0.035, 0.0085))) == pytest.approx((0.0, 0.0, 0.0))
    assert bracket_origin[0] + 0.038 > 0.0
    bracket_collision = robot.find("./link[@name='camera_tilt_link']/collision[@name='camera_tilt_bracket_collision']")
    assert _xyz(bracket_collision) == pytest.approx((0.0115, 0.0, 0.005))
    assert tuple(float(v) for v in bracket_collision.find('./geometry/box').attrib['size'].split()) == pytest.approx((0.043, 0.072, 0.030))

    mount = robot.find("./link[@name='camera_mount_link']")
    servo_case = mount.find("./visual[@name='camera_servo_case']")
    servo_collision = mount.find("./collision[@name='camera_servo_collision']")
    servo_center = pytest.approx((-0.008, -0.030, 0.0))
    assert _xyz(servo_case) == servo_center
    assert _xyz(servo_collision) == servo_center
    assert tuple(a + b for a, b in zip(_xyz(joints['camera_mount_joint']), _xyz(servo_case))) == pytest.approx(
        (0.082, -0.030, 0.115))
    assert servo_case.find('material').attrib['name'] == 'servo_case_pink'

    laser = robot.find("./link[@name='laser_frame']")
    plate = laser.find("./visual[@name='lidar_mount_plate_mesh']")
    standoff = laser.find("./visual[@name='lidar_front_left_standoff']")
    assert _xyz(plate)[2] == pytest.approx(-0.08205)
    assert _xyz(standoff)[2] == pytest.approx(-0.060)
    assert float(standoff.find('geometry/cylinder').attrib['length']) == pytest.approx(0.034)
    assert standoff.find('material').attrib['name'] == 'body_blue'
    plate_collision = laser.find("./collision[@name='lidar_mount_collision']")
    # 雷达扫描面离地 217 mm；STL 转正后厚 5 mm，支架底面贴 135 mm 顶板。
    assert 0.217 + _xyz(plate_collision)[2] - 0.0025 == pytest.approx(0.135)
    assert 0.217 + _xyz(plate_collision)[2] + 0.0025 == pytest.approx(
        0.217 + _xyz(standoff)[2] - 0.017)
    lidar_bottom = laser.find("./visual[@name='lidar_bottom_plate']")
    assert 0.217 + _xyz(lidar_bottom)[2] - 0.003 - 0.135 == pytest.approx(0.039)

    shell = robot.find("./link[@name='camera_link']/visual[@name='camera_silver_shell']")
    assert _xyz(shell)[1] == pytest.approx(-0.0175)
    # 用户量得外壳中心线与舵机轴 Y=0 对齐；外壳相对左红外光心回退 17.5 mm。
    assert _xyz(joints['camera_mount_joint'])[1] + _xyz(joints['camera_joint'])[1] + _xyz(shell)[1] == pytest.approx(0.0)
    camera = robot.find("./link[@name='camera_link']")
    apertures = {
        name: _xyz(camera.find(f"./visual[@name='camera_{name}']"))[1]
        for name in ('right_ir', 'projector', 'left_ir', 'rgb')
    }
    assert apertures == pytest.approx({
        'right_ir': -0.050, 'projector': -0.029,
        'left_ir': 0.0, 'rgb': 0.015,
    })

    expected_meshes = {
        'carcar_upper_structure.stl',
        'lidar_mount_plate.stl',
        'camera_servo_left_foot.stl',
        'camera_servo_right_foot.stl',
        'camera_tilt_bracket.stl',
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


def test_candidate_camera_extrinsic_does_not_move_mount_or_body(tmp_path):
    candidate = tmp_path / 'camera_candidate.yaml'
    candidate.write_text(
        'parent_frame: base_link\n'
        'child_frame: camera_link\n'
        'verified: false\n'
        'translation: {x: 0.201, y: -0.012, z: 0.155}\n'
        'rpy: {roll: 3.10, pitch: 0.08, yaw: -0.04}\n',
        encoding='utf-8')
    robot = _expanded_robot({'camera_extrinsics_file': str(candidate)})
    joints = {joint.attrib['name']: joint for joint in robot.findall('joint')}

    assert _xyz(joints['camera_joint']) == pytest.approx((0.111, -0.012, 0.040))
    assert _rpy(joints['camera_joint']) == pytest.approx((3.10, 0.08, -0.04))
    assert _xyz(joints['camera_mount_joint']) == pytest.approx((0.09, 0.0, 0.115))
    assert _rpy(joints['camera_mount_joint']) == (0.0, 0.0, 0.0)


def test_dynamic_camera_tilt_is_bounded_and_preserves_neutral_extrinsic():
    robot = _expanded_robot({'camera_tilt_dynamic': 'true'})
    joints = {joint.attrib['name']: joint for joint in robot.findall('joint')}
    tilt = joints['camera_tilt_joint']
    assert tilt.attrib['type'] == 'revolute'
    assert tilt.find('axis').attrib['xyz'] == '0 1 0'
    limits = tilt.find('limit').attrib
    assert float(limits['lower']) == pytest.approx(-0.968657735)
    assert float(limits['upper']) == pytest.approx(0.340339204)
    # 零倾角的链式平移仍等于独立 YAML 中的 base_link -> camera_link 初值。
    mount = _xyz(joints['camera_mount_joint'])
    camera = _xyz(joints['camera_joint'])
    assert tuple(a + b for a, b in zip(mount, camera)) == pytest.approx((0.150, 0.0175, 0.125))


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
    assert 0.03 + 0.100 + z + 0.130 == pytest.approx(0.135)
