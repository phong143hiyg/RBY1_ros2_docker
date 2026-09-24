#!/usr/bin/env python3
import argparse
import datetime
import json
import os
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET
import yaml
from ament_index_python.packages import get_package_share_directory, get_package_prefix


def command(*args):
    result = subprocess.run(args, capture_output=True, text=True)
    return {'exit_code': result.returncode, 'stdout': result.stdout.strip(), 'stderr': result.stderr.strip()}


def capture(output):
    vendor = Path('/opt/rby1_ros2_ws/src/rby1-ros2')
    config = Path(get_package_share_directory('rby1_moveit_m_1_2')) / 'config'
    description = Path(get_package_share_directory('rby1_description')) / 'urdf/rby1m/model_v1_2.urdf'
    root = ET.parse(description).getroot()
    mtc = None
    try:
        prefix = Path(get_package_prefix('moveit_task_constructor_core'))
        source = prefix.parents[1] / 'src/moveit_task_constructor'
        mtc = {'prefix': str(prefix), 'commit': command('git', '-C', str(source), 'rev-parse', 'HEAD')}
    except Exception as exc:
        mtc = {'available': False, 'reason': str(exc)}
    report = {
        'captured_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'host_ros': 'not installed; tests run in isolated Docker container',
        'ROS_DISTRO': os.environ.get('ROS_DISTRO'), 'ROS_DOMAIN_ID': os.environ.get('ROS_DOMAIN_ID'),
        'ROS_LOCALHOST_ONLY': os.environ.get('ROS_LOCALHOST_ONLY'), 'DISPLAY': os.environ.get('DISPLAY'),
        'sdk_commit': command('git', '-C', '/opt/rby1-sdk', 'rev-parse', 'HEAD'),
        'vendor_commit': command('git', '-C', str(vendor), 'rev-parse', 'HEAD'),
        'vendor_existing_changed_files': command('git', '-C', str(vendor), 'diff', '--name-only'),
        'debian_versions': command('dpkg-query', '-W', 'ros-humble-moveit*', 'ros-humble-ros2-control',
                                   'ros-humble-ros2-controllers', 'python3-pytest', 'libyaml-cpp-dev', 'nlohmann-json3-dev'),
        'mtc': mtc, 'model': root.attrib['name'], 'model_selection': 'fake demonstration only; physical model unknown',
        'urdf': str(description), 'srdf': str(config / 'RBY1_M_v1_2.srdf'),
        'kinematics': yaml.safe_load((config / 'kinematics.yaml').read_text()),
        'vendor_joint_limits': yaml.safe_load((config / 'joint_limits.yaml').read_text()),
        'vendor_controllers': yaml.safe_load((config / 'ros2_controllers.yaml').read_text()),
        'vendor_moveit_controllers': yaml.safe_load((config / 'moveit_controllers.yaml').read_text()),
        'gripper_urdf': {joint.attrib['name']: {'type': joint.attrib['type'],
                            'limit': joint.find('limit').attrib if joint.find('limit') is not None else None,
                            'mimic': joint.find('mimic').attrib if joint.find('mimic') is not None else None}
                        for joint in root.findall('joint') if 'gripper' in joint.attrib['name']},
        'specification': {'attachment_read': True, 'prompts-test-moveit2-rby1.md_found': False,
                          'protocol_common_fixture_available': False},
        'AGENTS': 'No applicable AGENTS.md found in repository or ancestor directories',
        'execution_enabled': False,
    }
    Path(output).write_text(json.dumps(report, indent=2))

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', default='/tmp/rby1-environment.json')
    capture(parser.parse_args().output)
