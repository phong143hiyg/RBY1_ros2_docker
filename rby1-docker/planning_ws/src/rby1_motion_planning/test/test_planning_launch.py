"""Opt-in launch_testing: launch GenericSystem, then exercise the actual NDJSON CLI."""
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
from ament_index_python.packages import get_package_share_directory

SHARE = Path(get_package_share_directory('rby1_motion_planning'))


def generate_test_description():
    description = IncludeLaunchDescription(PythonLaunchDescriptionSource(str(SHARE / 'launch/planning_test.launch.py')),
        launch_arguments={'model': 'm', 'moveit_package': 'rby1_moveit_m_1_2', 'rviz': 'false', 'contract_port': '0',
                          'port': '7451', 'report_dir': '/tmp/rby1-launch-test-audit'}.items())
    return launch.LaunchDescription([description, launch_testing.actions.ReadyToTest()])


class PlanningLaunch(unittest.TestCase):
    def test_ndjson_scenarios(self):
        lib = SHARE.parents[1] / 'lib/rby1_motion_planning'
        sys.path.insert(0, str(lib))
        from planning_cli import Client
        until = time.monotonic() + 60
        last = None
        while time.monotonic() < until:
            client = None
            try:
                client = Client(port=7451)
                state = client.call('get_state')
                scene = client.call('get_scene')
                if state.get('ok') and scene.get('ok') and len(scene['result']['scene']['objects']) == 5:
                    break
            except (OSError, ValueError, ConnectionError) as exc:
                last = str(exc)
            finally:
                if client:
                    client.close()
            time.sleep(.2)
        else:
            self.fail('Fake launch failed to become ready: ' + str(last))
        output = os.environ.get('RBY1_TEST_REPORT_DIR', '/tmp/rby1-launch-test-reports')
        result = subprocess.run([sys.executable, str(lib / 'run_scenarios.py'), '--port', '7451',
            '--fixtures', str(SHARE / 'fixtures/scenarios.json'), '--output', output,
            '--repetitions', os.environ.get('RBY1_TEST_REPETITIONS', '2')],
            capture_output=True, text=True, timeout=240)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


@launch_testing.post_shutdown_test()
class Shutdown(unittest.TestCase):
    def test_launch_shutdown(self, proc_info):
        # ROS nodes commonly return -2 on SIGINT; scenario result above determines correctness.
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15, 130])
