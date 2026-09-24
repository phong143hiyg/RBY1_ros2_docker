"""Wire compatibility checks against the exact Qt contract files.

These tests exercise protocol conversion, not MoveIt trajectory generation.
"""
import hashlib
import importlib.util
import asyncio
import json
from pathlib import Path
import sys
import time
import unittest

import yaml

package = Path(__file__).parents[1]
contract_root = Path(__file__).parents[4] / 'planning_protocol'
spec = importlib.util.spec_from_file_location('contract_v1', package / 'scripts/contract_v1.py')
wire = importlib.util.module_from_spec(spec)
spec.loader.exec_module(wire)
sys.path.insert(0, str(package / 'scripts'))
service_spec = importlib.util.spec_from_file_location('planning_contract_service', package / 'scripts/planning_contract_service.py')
service_module = importlib.util.module_from_spec(service_spec)
service_spec.loader.exec_module(service_module)


class ContractV1Test(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.fixture = json.loads((contract_root / 'fixtures/contract-v1.json').read_text())
        cls.doc = yaml.safe_load((package / 'config/test_scene.yaml').read_text())

    def test_exact_qt_sources(self):
        files = {'protocol-v1.md': '9b89c1b1f8a26068228f48ce4c5ea925a646496cdf9f8496b373861eb6dd829d',
                 'fixtures/contract-v1.json': '439e435361e1fb8ef649c143b77b21c00a2ba66b2385b2e5c35a25b942e52b93'}
        for name, expected in files.items():
            self.assertEqual(hashlib.sha256((contract_root / name).read_bytes()).hexdigest(), expected)

    def test_requests_from_qt_fixture(self):
        for command, sample in self.fixture['requests'].items():
            parsed = wire.parse_request((json.dumps(sample) + '\r\n').encode())
            self.assertEqual(parsed['command'], command)
        for data in (b'[]\n', b'not json\n', b'\xff\n'):
            with self.assertRaises(wire.ContractError):
                wire.parse_request(data)
        with self.assertRaises(wire.ContractError):
            wire.parse_request(b'{' + b' ' * wire.MAX_FRAME + b'}\n')

    def test_capabilities_use_verified_fake_identity(self):
        backend = {'model_id': 'ab' * 32, 'planning_frame': 'base',
                   'groups': ['right_arm'], 'tcp': 'ee_right',
                   'plan_pick_place': False, 'pick_place_errors': ['MIMIC_BOUNDS_CONFLICT']}
        result = wire.capabilities(backend)
        for field in self.fixture['capabilities']:
            self.assertIn(field, result)
        self.assertEqual(result['backend_mode'], 'fake_hardware')
        self.assertFalse(result['execution_enabled'])
        self.assertNotEqual(result['robot_model_id'], self.fixture['capabilities']['robot_model_id'])

    def test_scene_and_plan_payload(self):
        scene = wire.scene_payload(self.doc, 'sha256:test', 'RBY1_M_v1_2:sha256:test')
        for field in self.fixture['scene']:
            self.assertIn(field, scene)
        self.assertEqual({o['id'] for o in scene['objects']},
                         {'floor', 'table', 'obstacle_box', 'obstacle_column', 'target_object'})
        self.assertEqual(scene['pick_tcp_pose']['frame_id'], 'base')
        source = dict(self.fixture['requests']['plan_to_pose']['payload'])
        source.update(scene_revision=scene['scene_revision'], group='right_arm', tcp_frame='ee_right')
        source['goal_tcp_pose'] = scene['pick_tcp_pose']
        mapped = wire.plan_params('plan_to_pose', source, scene)
        self.assertEqual(mapped['tcp'], 'ee_right')
        self.assertEqual(mapped['pose']['orientation'], scene['pick_tcp_pose']['orientation_xyzw'])
        source['scene_revision'] = 'old'
        with self.assertRaises(wire.ContractError) as caught:
            wire.plan_params('plan_to_pose', source, scene)
        self.assertEqual(caught.exception.code, 'STALE_PLAN')

    def test_terminal_requires_real_validation(self):
        scene = wire.scene_payload(self.doc, 'sha256:test', 'RBY1_M_v1_2:sha256:test')
        metadata = {'plan_id': 'plan-real', 'planning_time_s': .1,
                    'start_state': {'right_arm_0': 0.0, 'right_arm_1': -.3},
                    'stages': [{'name': 'plan_to_pose', 'validation': {
                        'valid': True, 'duration_s': 1.2, 'waypoints': 4,
                        'resolution': .02, 'collision_samples': 80}}]}
        result = wire.terminal_result(metadata, scene)
        for field in self.fixture['result']:
            self.assertIn(field, result)
        self.assertEqual(result['validation']['simulated'], False)
        self.assertEqual(result['validation']['collision'], 'passed')
        metadata['stages'][0].pop('validation')
        with self.assertRaises(wire.ContractError):
            wire.terminal_result(metadata, scene)

    def test_error_codes_are_contract_codes(self):
        self.assertEqual(wire.error('PLAN_EXPIRED', 'expired')['code'], 'STALE_PLAN')
        self.assertEqual(wire.error('START_COLLISION', 'collision')['code'], 'START_IN_COLLISION')
        self.assertEqual(wire.error('GRASP_CONFIGURATION_INVALID', 'bad mimic')['code'], 'VALIDATION_FAILED')


class AdapterLifecycleTest(unittest.IsolatedAsyncioTestCase):
    async def test_ack_terminal_and_status_after_client_reconnect(self):
        doc = yaml.safe_load((package / 'config/test_scene.yaml').read_text())
        signature = {'objects': [{'id': o['id'], 'frame': 'base',
            'shapes': [{'type': 1 if o['type'] == 'box' else 3, 'dimensions': o['dimensions']}],
            'poses': [[0, 0, 0, 0, 0, 0, 1], o['pose']['position'] + o['pose']['orientation']]}
            for o in doc['objects']], 'attached': [], 'octomap': {'data': []}}

        class Backend:
            async def call(self, method, params=None, callback=None, timeout=8, wire_id=None):
                if method == 'capabilities':
                    result = {'model_id': 'ab' * 32, 'planning_frame': 'base',
                              'groups': ['right_arm'], 'tcp': 'ee_right', 'plan_pick_place': False}
                elif method == 'get_scene':
                    result = {'revision': 2, 'scene': signature}
                elif method == 'get_state':
                    result = {'state': {'right_arm_0': 0, 'right_arm_1': -.3}}
                elif method == 'plan_to_pose':
                    async def finish():
                        await asyncio.sleep(.01)
                        await callback({'type': 'event', 'terminal': True, 'ok': True,
                            'result': {'plan_id': 'plan-internal', 'planning_time_s': .2,
                                'start_state': {'right_arm_0': 0.0, 'right_arm_1': -.3},
                                'stages': [{'validation': {'valid': True, 'duration_s': 1.0,
                                    'waypoints': 3, 'resolution': .02, 'collision_samples': 50}}]}})
                    asyncio.create_task(finish())
                    return {'type': 'ack'}, wire_id
                else:
                    raise AssertionError(method)
                return {'type': 'response', 'ok': True, 'result': result}, wire_id or 'internal'

        class Writer:
            def __init__(self): self.messages = []
            def is_closing(self): return False
            def write(self, data): self.messages.append(json.loads(data))
            async def drain(self): pass

        server = service_module.ContractServer(Backend(), package / 'config/test_scene.yaml')
        await server.initialize()
        old = Writer()
        payload = {'scene_revision': server.scene['scene_revision'], 'group': 'right_arm',
                   'tcp_frame': 'ee_right', 'goal_tcp_pose': server.scene['pick_tcp_pose'],
                   'planning_timeout_s': 5, 'velocity_scale': .2, 'acceleration_scale': .2}
        await server.dispatch({'request_id': 'session-1', 'command': 'plan_to_pose', 'payload': payload}, old)
        await asyncio.sleep(.02)
        self.assertEqual([m['type'] for m in old.messages], ['response', 'event', 'event'])
        self.assertEqual([m['event_seq'] for m in old.messages[1:]], [1, 2])
        self.assertEqual(old.messages[-1]['status'], 'succeeded')
        self.assertFalse(old.messages[-1]['payload']['validation']['simulated'])
        new = Writer()
        await server.dispatch({'request_id': 'session-2', 'command': 'get_task_status',
                               'payload': {'target_request_id': 'session-1'}}, new)
        snapshot = new.messages[0]['payload']
        self.assertEqual(snapshot['request_id'], 'session-1')
        self.assertEqual(snapshot['event_seq'], 2)
        self.assertEqual(snapshot['status'], 'succeeded')
        server.tasks['uncertain'] = {'id': 'uncertain', 'command': 'plan_to_pose',
            'status': 'planning', 'updated': time.monotonic()}
        server.backend_lost()
        lost = Writer()
        await server.dispatch({'request_id': 'session-3', 'command': 'get_task_status',
                               'payload': {'target_request_id': 'uncertain'}}, lost)
        self.assertFalse(lost.messages[0]['ok'])
        self.assertEqual(lost.messages[0]['error']['code'], 'STATE_UNAVAILABLE')


if __name__ == '__main__':
    unittest.main()
