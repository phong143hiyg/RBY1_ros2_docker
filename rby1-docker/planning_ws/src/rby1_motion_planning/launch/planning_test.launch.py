"""Mandatory model/package; vendor configuration with verified GenericSystem only."""
from pathlib import Path
import hashlib
import json
import os
import xml.etree.ElementTree as ET
import yaml
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)
    model, package = arg('model'), arg('moveit_package')
    # DDS discovery for fake joint states must stay outside the command driver's domain.
    # An unset ROS_DOMAIN_ID is equivalent to the usual domain 0 default.
    if os.environ.get('ROS_DOMAIN_ID', '0') == '0':
        raise RuntimeError('Set a dedicated nonzero ROS_DOMAIN_ID for fake planning (for example 83).')
    # This integration was surveyed at the pinned vendor commit. Refuse unverified package/model combinations.
    verified = {'rby1_moveit_m_1_2': ('m', 'RBY1_M_v1_2')}
    if package not in verified or model != verified[package][0]:
        raise RuntimeError('Verified fake fixture requires model:=m moveit_package:=rby1_moveit_m_1_2. '
                           'Additional models require a recorded configuration audit and fixture calibration.')
    robot_name = verified[package][1]
    vendor = Path(get_package_share_directory(package))
    own = Path(get_package_share_directory('rby1_motion_planning'))
    # Explicit mapping overrides official demo's default false. Do not include driver/demo launch.
    config = (MoveItConfigsBuilder(robot_name, package_name=package)
              .robot_description(file_path=f'config/{robot_name}.urdf.xacro', mappings={
                  'use_fake_hardware': 'true', 'model': model,
                  'initial_positions_file': str(own / 'config/fake_initial_positions.yaml'),
                  'driver_namespace': 'planning_fake'})
              .planning_pipelines(pipelines=['ompl'])
              .to_moveit_configs())
    ompl_overlay = yaml.safe_load((own / 'config/ompl_planning.yaml').read_text())
    config.planning_pipelines['ompl'].update(ompl_overlay)
    urdf = config.robot_description['robot_description']
    root = ET.fromstring(urdf)
    plugins = [x.text.strip() for x in root.findall('./ros2_control/hardware/plugin')]
    if not plugins or any(x != 'mock_components/GenericSystem' for x in plugins):
        raise RuntimeError(f'Fake hardware audit failed: {plugins}')
    # Local semantic overlay only: never edits installed vendor SRDF.
    original = config.robot_description_semantic['robot_description_semantic']
    srdf = ET.fromstring(original)
    overlay_diff = []
    if not srdf.findall('end_effector'):
        ET.SubElement(srdf, 'end_effector', name='rby1_right_tool', parent_link='ee_right',
                      group='gripper_r', parent_group='right_arm')
        overlay_diff.append('<end_effector name="rby1_right_tool" parent_link="ee_right" group="gripper_r" parent_group="right_arm"/>')
    config.robot_description_semantic['robot_description_semantic'] = ET.tostring(srdf, encoding='unicode')
    # Manufacturer acceleration limits are absent in vendor config. Explicit fake-only overlay for retiming.
    limits = config.joint_limits['robot_description_planning']['joint_limits']
    for name, value in limits.items():
        value['has_acceleration_limits'] = True
        value['max_acceleration'] = 0.1 if 'finger' in name else 1.0
    common = config.to_dict()
    common['moveit_controller_manager'] = 'rby1_motion_planning/PlanningOnlyControllerManager'
    common['moveit_manage_controllers'] = False
    common.pop('moveit_simple_controller_manager', None)
    identity = hashlib.sha256((urdf + common['robot_description_semantic'] + json.dumps(limits, sort_keys=True)).encode()).hexdigest()
    capability_library = str(Path(get_package_prefix('moveit_ros_move_group')) / 'lib/libmoveit_move_group_default_capabilities.so')
    report_dir = Path(arg('report_dir'))
    report_dir.mkdir(parents=True, exist_ok=True)
    (report_dir / 'launch_audit.json').write_text(json.dumps({
        'model': model, 'moveit_package': package, 'model_id': identity,
        'use_fake_hardware': True, 'hardware_plugins': plugins, 'execution_enabled': False,
        'move_group_environment': {'LD_PRELOAD': capability_library},
        'shutdown_workaround': 'Keep MoveIt 2.5.9 capability code mapped through node callback-group destruction; vendor files unchanged',
        'ompl_overlay': ompl_overlay,
        'vendor_srdf_sha256': hashlib.sha256(original.encode()).hexdigest(),
        'srdf_overlay_added': overlay_diff,
        'joint_limit_overlay': {'has_acceleration_limits': True, 'arm_rad_s2': 1.0, 'finger_m_s2': 0.1,
                               'purpose': 'illustrative fake-only retiming; not manufacturer limits'},
        'moveit_controller_manager': common['moveit_controller_manager'], 'controllers': ['joint_state_broadcaster'], 'joint_state_owners': ['joint_state_broadcaster'],
        'scene_file': arg('scene_file'), 'fake_initial_positions_file': str(own / 'config/fake_initial_positions.yaml')}, indent=2))
    # Keep capability code mapped until process exit: 2.5.9 otherwise unloads it while
    # the node still owns weak callback-group control blocks. Audited process-local workaround.
    # A single broadcaster owns joint states. No trajectory/gripper command controller is activated.
    control = Node(package='controller_manager', executable='ros2_control_node',
                   parameters=[config.robot_description, str(vendor / 'config/ros2_controllers.yaml')], output='screen')
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               parameters=[config.robot_description], output='screen')
    move_group = Node(package='moveit_ros_move_group', executable='move_group', parameters=[common, {
        'allow_trajectory_execution': False, 'publish_robot_description': True,
        'publish_robot_description_semantic': True,
        'publish_planning_scene': True, 'publish_geometry_updates': True,
        'publish_state_updates': True, 'publish_transforms_updates': True,
        # Remove execute action capabilities; planning and scene services remain.
        'disable_capabilities': 'move_group/MoveGroupExecuteTrajectoryAction move_group/MoveGroupPickPlaceAction'}],
        additional_env={'LD_PRELOAD': capability_library}, output='screen')
    loader = Node(package='rby1_motion_planning', executable='scene_loader', parameters=[common, {
        'scene_file': arg('scene_file')}], output='screen')
    worker = Node(package='rby1_motion_planning', executable='planning_worker', parameters=[common, {
        'model_id': identity, 'arm_group': 'right_arm', 'tcp_link': 'ee_right',
        'scene_file': arg('scene_file'), 'task_parameters_file': str(own / 'config/task_parameters.yaml')}], output='screen')
    transport = Node(package='rby1_motion_planning', executable='planning_service.py', parameters=[{
        'bind': arg('bind'), 'port': int(arg('port'))}], output='screen')
    contract_port = int(arg('contract_port'))
    contract_executable = str(Path(get_package_prefix('rby1_motion_planning')) /
                              'lib/rby1_motion_planning/planning_contract_service.py')
    contract = ExecuteProcess(cmd=[contract_executable, '--bind', arg('contract_bind'),
        '--port', str(contract_port), '--backend-port', arg('port'),
        '--scene-file', arg('scene_file')], output='screen')
    actions = [control, rsp, move_group,
        Node(package='controller_manager', executable='spawner', arguments=['joint_state_broadcaster', '--controller-manager-timeout', '60'], output='screen'),
        loader, worker, transport]
    if contract_port:
        actions.append(contract)
    if arg('rviz').lower() == 'true':
        actions.append(Node(package='rviz2', executable='rviz2', arguments=['-d', str(own / 'config/planning.rviz')], parameters=[common], output='screen'))
    for process in (control, move_group, loader, worker, transport) + ((contract,) if contract_port else ()):
        actions.append(RegisterEventHandler(OnProcessExit(target_action=process,
            on_exit=[EmitEvent(event=Shutdown(reason='Critical planning process exited'))])))
    return actions


def generate_launch_description():
    own = Path(get_package_share_directory('rby1_motion_planning'))
    return LaunchDescription([
        DeclareLaunchArgument('model', description='Required fake demonstration model; verified value m'),
        DeclareLaunchArgument('moveit_package', description='Required vendor package; verified rby1_moveit_m_1_2'),
        DeclareLaunchArgument('scene_file', default_value=str(own / 'config/test_scene.yaml')),
        DeclareLaunchArgument('report_dir', default_value='/tmp/rby1-planning-reports'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('bind', default_value='127.0.0.1'),
        DeclareLaunchArgument('port', default_value='7447'),
        DeclareLaunchArgument('contract_bind', default_value='127.0.0.1'),
        DeclareLaunchArgument('contract_port', default_value='8082'), OpaqueFunction(function=setup)])
