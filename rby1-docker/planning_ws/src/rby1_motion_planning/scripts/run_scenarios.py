#!/usr/bin/env python3
"""Real TCP -> ROS -> MoveIt scenarios, with persisted calibration and results."""
import argparse
import copy
import csv
import json
from pathlib import Path
import statistics
import time
from planning_cli import Client

class Suite:
    def __init__(self, client, output, repetitions, fixtures):
        self.client, self.output, self.repetitions = client, Path(output), repetitions
        self.fixtures = json.loads(Path(fixtures).read_text())
        self.results = []
        self.output.mkdir(parents=True, exist_ok=True)
        self.artifacts = {}

    def call(self, method, params=None):
        response = self.client.call(method, params)
        assert response.get('execution_enabled') is False, response
        return response

    @staticmethod
    def ok(response):
        assert response.get('ok') is True, response
        return response['result']

    @staticmethod
    def error(response, *codes):
        assert response.get('ok') is False, response
        assert response['error']['code'] in codes, response
        assert 'result' not in response, response
        return response

    def record(self, name, fn):
        begin = time.monotonic()
        try:
            details = fn()
            result = {'name': name, 'status': 'PASS', 'details': details}
        except Exception as exc:
            result = {'name': name, 'status': 'FAIL', 'error': str(exc)}
        result['elapsed_s'] = time.monotonic() - begin
        self.results.append(result)
        print(f"{result['status']} {name} {result['elapsed_s']:.3f}s", flush=True)
        self.save()
        return result['status'] == 'PASS'

    def skip(self, name, reason):
        self.results.append({'name': name, 'status': 'SKIP', 'reason': reason, 'elapsed_s': 0})
        self.save()

    def save(self):
        (self.output / 'scenarios.json').write_text(json.dumps({'results': self.results, 'artifacts': self.artifacts}, indent=2))
        with (self.output / 'scenarios.csv').open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=['name', 'status', 'elapsed_s'])
            writer.writeheader()
            for row in self.results:
                writer.writerow({key: row[key] for key in writer.fieldnames})
        with (self.output / 'planning_trials.csv').open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=['scenario', 'trial', 'ok', 'error_code', 'planning_time_s', 'duration_s', 'waypoints', 'collision_samples'])
            writer.writeheader()
            for scenario in ('baseline_trials', 'obstacle_trials'):
                for index, response in enumerate(self.artifacts.get(scenario, [])):
                    validation = response.get('result', {}).get('stages', [{}])[0].get('validation', {})
                    writer.writerow({'scenario': scenario, 'trial': index + 1, 'ok': response.get('ok'),
                        'error_code': response.get('error', {}).get('code'),
                        'planning_time_s': response.get('result', {}).get('planning_time_s'),
                        'duration_s': validation.get('duration_s'), 'waypoints': validation.get('waypoints'),
                        'collision_samples': validation.get('collision_samples')})
        (self.output / 'protocol_trace.ndjson').write_text(''.join(json.dumps(x) + '\n' for x in self.client.messages))

    def scene(self, objects):
        return self.ok(self.call('load_scene', {'scene': {'frame': self.frame, 'objects': objects}}))

    def box(self, name, position, dimensions):
        return {'id': name, 'type': 'box', 'dimensions': dimensions,
                'pose': {'position': position, 'orientation': [0, 0, 0, 1]}}

    def pose_params(self, pose):
        return {'frame': self.frame, 'pose': pose, 'deadline_s': 15}

    def calibrate(self):
        state = self.ok(self.call('get_state'))
        self.start = state['state']
        self.start_pose = state['tcp_pose']
        self.frame = state['frame']
        self.artifacts['start'] = state
        self.scene([])
        info = self.ok(self.call('check_state'))
        assert info['bounds_valid'] and not info['collision'], info
        candidates = self.fixtures['goal_offsets']
        for delta in candidates:
            goal = copy.deepcopy(self.start_pose)
            goal['position'] = [x + d for x, d in zip(goal['position'], delta)]
            probe = self.call('cartesian_probe', self.pose_params(goal))
            if not probe.get('ok') or not probe['result']['complete']:
                continue
            plan = self.call('plan_to_pose', self.pose_params(goal))
            if plan.get('ok'):
                self.goal, self.baseline = goal, plan['result']
                self.artifacts['baseline'] = self.baseline
                self.artifacts['goal'] = goal
                self.artifacts['bare_cartesian'] = probe['result']
                return {'goal': goal, 'plan_id': self.baseline['plan_id'], 'cartesian_fraction': probe['result']['fraction']}
        raise AssertionError('No reachable clear Cartesian goal among configured offsets; fixture calibration failed')

    def baseline_repeats(self):
        trials = []
        for _ in range(self.repetitions):
            trials.append(self.call('plan_to_pose', self.pose_params(self.goal)))
        successes = [x for x in trials if x.get('ok')]
        self.artifacts['baseline_trials'] = trials
        assert len(successes) == self.repetitions, trials
        return {'success_rate': len(successes) / len(trials), 'repetitions': len(trials),
                'planning_time_mean_s': statistics.mean(x['result']['planning_time_s'] for x in successes)}

    def obstacle_detour(self):
        # Place an actual obstacle on a collision-free bare Cartesian sample, then require collision rejection and a full OMPL plan.
        bare = self.artifacts['bare_cartesian']['samples']
        candidates = [(index, side) for side in self.fixtures['obstacle_sizes']
                      for index in (len(bare)//2, len(bare)//3, 2*len(bare)//3)]
        attempts = []
        for index, side in candidates:
            sample = bare[index]
            obstacle = self.box('straight_path_blocker', sample['tcp_pose']['position'], [side] * 3)
            self.scene([obstacle])
            start_info = self.ok(self.call('check_state'))
            end_info = self.ok(self.call('check_state', {'state': bare[-1]['state']}))
            midpoint_info = self.ok(self.call('check_state', {'state': sample['state']}))
            if start_info['collision'] or end_info['collision'] or not midpoint_info['collision']:
                continue
            probe = self.ok(self.call('cartesian_probe', self.pose_params(self.goal)))
            if probe['complete'] or probe['collision_rejections'] == 0:
                continue
            response = self.call('plan_to_pose', self.pose_params(self.goal))
            attempts.append({'obstacle': obstacle, 'probe': probe, 'response': response})
            if response.get('ok'):
                self.obstacle = obstacle
                self.artifacts['obstacle_scene'] = {'frame': self.frame, 'objects': [obstacle]}
                self.artifacts['obstacle_attempts'] = attempts
                trials = [response] + [self.call('plan_to_pose', self.pose_params(self.goal)) for _ in range(self.repetitions - 1)]
                self.artifacts['obstacle_trials'] = trials
                assert all(x.get('ok') for x in trials), trials
                return {'bare_fraction': 1.0, 'blocked_fraction': probe['fraction'],
                        'collision_rejections': probe['collision_rejections'], 'midpoint_contacts': midpoint_info['contacts'],
                        'success_rate': sum(x.get('ok') for x in trials)/len(trials),
                        'planning_time_mean_s': statistics.mean(x['result']['planning_time_s'] for x in trials),
                        'obstacle': obstacle}
        self.artifacts['obstacle_attempts'] = attempts
        raise AssertionError('No calibrated fixture proving blocked Cartesian path plus successful OMPL detour')

    @staticmethod
    def rotate(quaternion, vector):
        x, y, z, w = quaternion
        vx, vy, vz = vector
        tx, ty, tz = 2*(y*vz-z*vy), 2*(z*vx-x*vz), 2*(x*vy-y*vx)
        return [vx+w*tx+y*tz-z*ty, vy+w*ty+z*tx-x*tz, vz+w*tz+x*ty-y*tx]

    def attached_passage(self):
        bare = self.artifacts['bare_cartesian']['samples']
        payload = self.fixtures['virtual_payload']
        attempts = []
        for index in (len(bare)//2, len(bare)//3, 2*len(bare)//3):
            sample = bare[index]
            for lateral in (.16, -.16, .13, -.13):
                offset = self.rotate(sample['tcp_pose']['orientation'], [0, lateral, -.18])
                wall = self.box('payload_wall', [a+b for a, b in zip(sample['tcp_pose']['position'], offset)], [.04]*3)
                self.scene([wall])
                before = self.ok(self.call('get_scene'))
                bare_probe = self.ok(self.call('cartesian_probe', self.pose_params(self.goal)))
                if not bare_probe['complete']:
                    continue
                start_payload = self.ok(self.call('check_state', {'virtual_attachment': payload}))
                mid_payload = self.ok(self.call('check_state', {'state': sample['state'], 'virtual_attachment': payload}))
                if start_payload['collision'] or not mid_payload['collision']:
                    continue
                contacts = mid_payload['contacts']
                if not any(set(pair) == {'diagnostic_payload', 'payload_wall'} for pair in contacts):
                    continue
                attached_probe = self.ok(self.call('cartesian_probe', dict(self.pose_params(self.goal), virtual_attachment=payload)))
                attempts.append({'wall': wall, 'midpoint_contacts': contacts, 'bare': bare_probe, 'attached': attached_probe})
                if not attached_probe['complete'] and attached_probe['collision_rejections'] > 0:
                    after = self.ok(self.call('get_scene'))
                    assert before == after and after['scene']['attached'] == [], (before, after)
                    self.artifacts['rby1_virtual_payload'] = {'payload': payload, 'wall': wall, 'attempts': attempts,
                                                            'grasp_claim': False, 'live_scene_unchanged': True}
                    return {'bare_fraction': bare_probe['fraction'], 'attached_fraction': attached_probe['fraction'],
                            'attached_collision_rejections': attached_probe['collision_rejections'],
                            'midpoint_contacts': contacts, 'live_scene_unchanged': True, 'grasp_claim': False}
        self.artifacts['virtual_payload_attempts'] = attempts
        raise AssertionError('RBY1 virtual payload passage calibration failed; no grasp success claimed')

    def unchanged(self):
        self.scene([])
        before = self.ok(self.call('get_scene'))
        state_before = self.ok(self.call('get_state'))
        plan = self.ok(self.call('plan_to_pose', self.pose_params(self.goal)))
        self.ok(self.call('preview', {'plan_id': plan['plan_id']}))
        after = self.ok(self.call('get_scene'))
        state_after = self.ok(self.call('get_state'))
        assert before == after, (before, after)
        assert state_before['state'] == state_after['state'], (state_before, state_after)
        return {'plan_id': plan['plan_id'], 'scene_unchanged': True, 'live_joints_unchanged': True}

    def invalidation(self):
        self.scene([])
        plan = self.ok(self.call('plan_to_pose', self.pose_params(self.goal)))
        old_revision = plan['revision']
        reset = self.scene([])
        assert reset['revision'] > old_revision
        return self.error(self.call('preview', {'plan_id': plan['plan_id']}), 'PLAN_NOT_FOUND', 'PLAN_STALE')

    def cancel(self):
        self.scene([])
        params = self.pose_params({'position': [9, 9, 9], 'orientation': [0, 0, 0, 1]})
        task = self.client.send('plan_to_pose', params)
        ack = self.client.receive()
        assert ack['id'] == task and ack['type'] == 'ack', ack
        busy_id = self.client.send('plan_to_pose', params)
        cancel_id = self.client.send('cancel', {'task_id': task})
        terminal, cancellation, busy = None, None, None
        while terminal is None or cancellation is None or busy is None:
            response = self.client.receive()
            if response['id'] == task and response.get('terminal'):
                assert terminal is None, response
                terminal = response
            elif response['id'] == cancel_id:
                cancellation = response
            elif response['id'] == busy_id:
                busy = response
        self.error(busy, 'BUSY')
        self.error(terminal, 'CANCELLED')
        assert terminal['state'] == 'cancelled' and terminal['worker_stopped'] is True
        status = self.ok(self.call('get_task_status', {'task_id': task}))
        assert status['state'] == 'cancelled' and status['id'] == task, status
        # Worker has stopped and accepts the next task.
        self.ok(self.call('plan_to_pose', self.pose_params(self.goal)))
        return {'ack': ack, 'cancel': cancellation, 'terminal': terminal, 'status': status, 'busy': busy}

    def negative_goal(self):
        # A box at the goal TCP must cover all possible IK states of this TCP and clear the current state.
        self.scene([self.box('goal_blocker', self.goal['position'], [.07] * 3)])
        start = self.ok(self.call('check_state'))
        assert not start['collision'], start
        return self.error(self.call('plan_to_pose', self.pose_params(self.goal)), 'GOAL_COLLISION')

    def negative_start(self):
        self.scene([self.box('start_blocker', self.start_pose['position'], [.1] * 3)])
        return self.error(self.call('plan_to_pose', self.pose_params(self.goal)), 'START_COLLISION')

    def reset_repeat(self):
        snapshots = []
        for _ in range(self.repetitions):
            result = self.ok(self.call('reset_scene'))
            snapshots.append(result['scene'])
        assert all(x == snapshots[0] for x in snapshots), snapshots
        assert snapshots[0]['attached'] == []
        return {'repetitions': self.repetitions, 'objects': len(snapshots[0]['objects']), 'clean_attachments': True}

    def protocol_cases(self):
        request = self.client.send('capabilities', request_id='unique-protocol-id')
        first = self.client.receive()
        assert first['id'] == request and first['ok'], first
        self.client.send('capabilities', request_id='unique-protocol-id')
        duplicate = self.client.receive()
        self.error(duplicate, 'DUPLICATE_ID')
        forbidden = self.call('execute', {'plan_id': 'anything'})
        self.error(forbidden, 'UNKNOWN_METHOD')
        one = self.client.send('get_scene')
        two = self.client.send('capabilities')
        responses = [self.client.receive(), self.client.receive()]
        assert {x['id'] for x in responses} == {one, two}, responses
        assert all(x.get('ok') and x['execution_enabled'] is False for x in responses)
        self.client.file.write(b'{"v":2,"id":"invalid-version","method":"capabilities"}\n')
        self.client.file.flush()
        version = self.client.receive()
        self.error(version, 'UNSUPPORTED_VERSION')
        assert version['id'] == 'invalid-version'
        return {'duplicate': duplicate, 'forbidden': forbidden, 'version': version, 'correlation_ids': [one, two]}

    def run(self):
        self.caps = self.ok(self.call('capabilities'))
        self.artifacts['capabilities'] = self.caps
        if not self.record('baseline_calibration', self.calibrate):
            self.skip('dependent_scenarios', 'Baseline calibration failed; no fake successful trajectory generated')
            return 1
        self.record('baseline_reachable_repeats', self.baseline_repeats)
        self.record('plan_preview_live_scene_and_joints_unchanged', self.unchanged)
        self.record('obstacle_blocks_straight_but_ompl_detours', self.obstacle_detour)
        self.record('rby1_attached_object_passage_virtual_fixture', self.attached_passage)
        self.record('goal_collision', self.negative_goal)
        self.record('start_collision', self.negative_start)
        self.scene([])
        self.record('unreachable', lambda: self.error(self.call('plan_to_pose', self.pose_params(
            self.fixtures['unreachable_pose'])), 'IK_UNREACHABLE'))
        self.record('joint_limit', lambda: self.error(self.call('plan_to_pose', dict(self.pose_params(self.goal),
            start_state=self.fixtures['invalid_start_state'])), 'JOINT_LIMIT'))
        self.record('missing_frame', lambda: self.error(self.call('plan_to_pose', dict(self.pose_params(self.goal),
            frame=self.fixtures['missing_frame'])), 'INVALID_FRAME'))
        self.record('deadline_timeout', lambda: self.error(self.call('plan_to_pose', dict(self.pose_params(self.goal),
            deadline_s=0.000001)), 'TIMEOUT'))
        self.record('cancel_busy_status_correlation', self.cancel)
        self.record('protocol_ids_duplicate_and_execution_rejection', self.protocol_cases)
        self.record('scene_revision_plan_invalidation', self.invalidation)
        self.record('reset_repeat', self.reset_repeat)
        if not self.caps['plan_pick_place']:
            self.record('pick_place_explicit_capability_error', lambda: self.error(self.call('plan_pick_place'), 'GRASP_CONFIGURATION_INVALID'))
            for name in ('rby1_object_pose_after_detach', 'rby1_mtc_stage_sequence'):
                self.skip(name, 'RBY1 gripper model invalid: ' + str(self.caps['pick_place_errors']) + '. Generic geometry checks run in C++ validator tests.')
        else:
            def task_case():
                result = self.ok(self.call('plan_pick_place', self.fixtures['pick_place']))
                self.artifacts['pick_place'] = result
                return result
            self.record('rby1_mtc_stage_sequence', task_case)
        self.save()
        return 1 if any(x['status'] == 'FAIL' for x in self.results) else 0

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=7447)
    parser.add_argument('--output', default='/tmp/rby1-planning-reports')
    parser.add_argument('--repetitions', type=int, default=3)
    parser.add_argument('--fixtures', default=str(Path(__file__).parents[2] / 'share/rby1_motion_planning/fixtures/scenarios.json'))
    args = parser.parse_args()
    if args.repetitions < 1:
        parser.error('repetitions must be >= 1')
    client = Client(args.host, args.port)
    try:
        raise SystemExit(Suite(client, args.output, args.repetitions, args.fixtures).run())
    finally:
        client.close()
