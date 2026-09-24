"""Qt planning_protocol/protocol-v1.md wire format over the existing MoveIt worker.

The backend's private ROS protocol is intentionally kept behind this adapter.
"""
import hashlib
import json
import math

MAX_FRAME = 1048576
COMMANDS = ('get_capabilities', 'load_test_scene', 'get_scene', 'plan_to_pose',
            'plan_pick_place', 'preview_plan', 'get_task_status', 'cancel_planning')
SCENARIOS = ('pick_place_obstacles', 'baseline')
ERROR_CODES = {
    'INVALID_FRAME': 'TF_UNAVAILABLE', 'SCENE_TIMEOUT': 'STATE_UNAVAILABLE',
    'STATE_TIMEOUT': 'STATE_UNAVAILABLE', 'TASK_NOT_FOUND': 'STATE_UNAVAILABLE',
    'START_COLLISION': 'START_IN_COLLISION', 'GOAL_COLLISION': 'GOAL_IN_COLLISION',
    'IK_UNREACHABLE': 'PLANNING_FAILED', 'IK_SOLVER_UNAVAILABLE': 'PLANNING_FAILED',
    'PLAN_NOT_FOUND': 'STALE_PLAN', 'PLAN_EXPIRED': 'STALE_PLAN',
    'PLAN_STALE': 'STALE_PLAN', 'START_STATE_CHANGED': 'STALE_PLAN',
    'SCENE_CHANGED': 'STALE_PLAN', 'GRASP_CONFIGURATION_INVALID': 'VALIDATION_FAILED',
    'CANCELLED': 'PLANNING_FAILED',
}
VALID_ERRORS = {'INVALID_REQUEST', 'UNSUPPORTED_COMMAND', 'BUSY', 'TF_UNAVAILABLE',
                'STATE_UNAVAILABLE', 'START_IN_COLLISION', 'GOAL_IN_COLLISION',
                'NO_IK', 'PLANNING_FAILED', 'CARTESIAN_INCOMPLETE',
                'VALIDATION_FAILED', 'TIMEOUT', 'STALE_PLAN'}


class ContractError(ValueError):
    def __init__(self, code, message):
        super().__init__(message)
        self.code = code


def parse_request(line):
    if len(line.rstrip(b'\r\n')) > MAX_FRAME:
        raise ContractError('INVALID_REQUEST', 'NDJSON frame exceeds 1,048,576 bytes')
    try:
        request = json.loads(line.decode('utf-8'), parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value)))
    except (UnicodeError, ValueError) as exc:
        raise ContractError('INVALID_REQUEST', 'Invalid UTF-8 JSON object') from exc
    if not isinstance(request, dict) or request.get('protocol_version') != 1 or type(request.get('protocol_version')) is not int:
        raise ContractError('INVALID_REQUEST', 'Expected protocol_version 1 and object')
    if request.get('type') != 'request' or not isinstance(request.get('request_id'), str) or not request['request_id']:
        raise ContractError('INVALID_REQUEST', 'Expected request type and nonempty request_id')
    if len(request['request_id']) > 128 or not isinstance(request.get('command'), str):
        raise ContractError('INVALID_REQUEST', 'Invalid request_id or command')
    if request['command'] not in COMMANDS:
        raise ContractError('UNSUPPORTED_COMMAND', request['command'])
    if not isinstance(request.get('payload'), dict):
        raise ContractError('INVALID_REQUEST', 'payload must be an object')
    return request


def envelope(request_id, command, kind, payload=None, **extra):
    result = {'protocol_version': 1, 'type': kind, 'request_id': request_id,
              'command': command, 'payload': payload if payload is not None else {}}
    result.update(extra)
    return result


def error(code, message, stage='request', details=None):
    canonical = ERROR_CODES.get(code, code)
    if canonical not in VALID_ERRORS:
        canonical = 'VALIDATION_FAILED' if 'VALID' in code or 'COLLISION' in code else 'PLANNING_FAILED'
    return {'code': canonical, 'message': str(message), 'stage': stage,
            'details': details or {'backend_code': code}}


def pose_to_backend(pose, frame):
    if not isinstance(pose, dict) or pose.get('frame_id') != frame:
        raise ContractError('TF_UNAVAILABLE', 'Pose frame must equal planning frame')
    xyz, quat = pose.get('position'), pose.get('orientation_xyzw')
    if not isinstance(xyz, list) or not isinstance(quat, list) or len(xyz) != 3 or len(quat) != 4:
        raise ContractError('INVALID_REQUEST', 'Pose requires position[3], orientation_xyzw[4]')
    if any(type(v) not in (int, float) or not math.isfinite(v) for v in xyz + quat):
        raise ContractError('INVALID_REQUEST', 'Pose values must be finite')
    if abs(sum(v * v for v in quat) - 1) > .002001:
        raise ContractError('INVALID_REQUEST', 'Quaternion norm must be 1')
    return {'position': xyz, 'orientation': quat}


def fixture_pose(pose, frame):
    return {'frame_id': frame, 'position': pose['position'],
            'orientation_xyzw': pose['orientation']}


def scenario_doc(source, scenario):
    if scenario not in SCENARIOS:
        raise ContractError('INVALID_REQUEST', 'Unknown scenario_id')
    doc = dict(source)
    if scenario == 'baseline':
        doc['objects'] = [obj for obj in source['objects'] if obj['id'] not in ('obstacle_box', 'obstacle_column')]
    return doc


def scene_payload(doc, revision, model_id, group='right_arm', tcp='ee_right'):
    frame = doc['frame']
    roles = {'floor': 'support', 'table': 'support', 'target_object': 'target'}
    objects = [{'id': o['id'], 'role': roles.get(o['id'], 'obstacle'), 'geometry': o['type'],
                'dimensions_m': o['dimensions'], 'pose': fixture_pose(o['pose'], frame)}
               for o in doc['objects']]
    markers = [{'label': label, 'pose': fixture_pose(pose, frame)}
               for label, pose in doc['markers'].items()]
    return {'schema_version': 1, 'units': {'length': 'm', 'angle': 'rad', 'quaternion': 'xyzw'},
            'scenario_id': doc.get('scenario_id', 'pick_place_obstacles'),
            'scene_revision': revision, 'robot_model_id': model_id,
            'frame_id': frame, 'group': group, 'tcp_frame': tcp,
            'objects': objects, 'markers': markers,
            'pick_tcp_pose': fixture_pose(doc['markers']['pick TCP'], frame),
            'place_object_pose': fixture_pose(doc['markers']['place object'], frame),
            'cartesian_directions': doc['cartesian_directions'], 'defaults': doc['defaults']}


def revision_of(backend_scene, state):
    source = json.dumps({'revision': backend_scene['revision'], 'scene': backend_scene['scene'],
                         'state': state}, sort_keys=True, separators=(',', ':'))
    return 'sha256:' + hashlib.sha256(source.encode()).hexdigest()


def verify_scene(doc, backend_scene):
    signature = backend_scene['scene']
    if signature.get('attached') or signature.get('octomap', {}).get('data'):
        raise ContractError('STATE_UNAVAILABLE', 'Live MoveIt scene has unreviewed attachments or octomap')
    expected = {o['id']: o for o in doc['objects']}
    actual = {o['id']: o for o in signature['objects']}
    if expected.keys() != actual.keys():
        raise ContractError('STATE_UNAVAILABLE', 'Live MoveIt scene differs from selected fixture')
    for ident, object_doc in expected.items():
        shapes = actual[ident]['shapes']
        shape_type = 1 if object_doc['type'] == 'box' else 3
        if len(shapes) != 1 or shapes[0]['type'] != shape_type or list(shapes[0]['dimensions']) != list(object_doc['dimensions']):
            raise ContractError('STATE_UNAVAILABLE', 'Live MoveIt geometry differs: ' + ident)
        expected_pose = object_doc['pose']['position'] + object_doc['pose']['orientation']
        actual_pose = actual[ident]['poses'][-1]
        if actual[ident]['frame'] != doc['frame'] or len(actual_pose) != 7 or any(
                abs(a - b) > 1e-6 for a, b in zip(actual_pose, expected_pose)):
            raise ContractError('STATE_UNAVAILABLE', 'Live MoveIt pose or frame differs: ' + ident)


def capabilities(backend, ttl=120):
    model_hash = backend['model_id']
    return {'backend_mode': 'fake_hardware', 'model': 'RBY1_M', 'model_version': '1.2',
            'robot_model_id': 'RBY1_M_v1_2:sha256:' + model_hash,
            'planning_frame': backend['planning_frame'], 'groups': backend['groups'],
            'tcp_mappings': [{'group': 'right_arm', 'tcp_frame': backend['tcp'], 'link': backend['tcp']}],
            'supported_commands': [c for c in COMMANDS if c != 'plan_pick_place' or backend.get('plan_pick_place')],
            'scenarios': list(SCENARIOS),
            'execution_enabled': False, 'max_frame_bytes': MAX_FRAME,
            'suggested_planning_timeout_s': 30, 'max_planning_timeout_s': 120,
            'plan_ttl_s': ttl, 'task_status_ttl_s': 300,
            'plan_pick_place_available': bool(backend.get('plan_pick_place')),
            'pick_place_errors': backend.get('pick_place_errors', [])}


def plan_params(command, payload, scene):
    if payload.get('scene_revision') != scene['scene_revision']:
        raise ContractError('STALE_PLAN', 'Scene revision changed')
    if payload.get('group') != scene['group'] or payload.get('tcp_frame') != scene['tcp_frame']:
        raise ContractError('INVALID_REQUEST', 'Group or TCP differs from scene snapshot')
    timeout = payload.get('planning_timeout_s')
    vs, accel = payload.get('velocity_scale'), payload.get('acceleration_scale')
    if any(type(v) not in (int, float) or not math.isfinite(v) for v in (timeout, vs, accel)):
        raise ContractError('INVALID_REQUEST', 'Timeout and scaling must be finite numbers')
    if not 0 < timeout <= 120 or not 0 < vs <= 1 or not 0 < accel <= 1:
        raise ContractError('INVALID_REQUEST', 'Timeout or scaling outside allowed range')
    result = {'group': scene['group'], 'tcp': scene['tcp_frame'], 'frame': scene['frame_id'],
              'deadline_s': timeout, 'velocity_scaling': vs, 'acceleration_scaling': accel}
    if command == 'plan_to_pose':
        result['pose'] = pose_to_backend(payload.get('goal_tcp_pose'), scene['frame_id'])
    else:
        if payload.get('object_id') not in {o['id'] for o in scene['objects'] if o['role'] == 'target'}:
            raise ContractError('INVALID_REQUEST', 'Unknown target object')
        result['pick_tcp'] = pose_to_backend(payload.get('pick_tcp_pose'), scene['frame_id'])
        result['place_object'] = pose_to_backend(payload.get('place_object_pose'), scene['frame_id'])
        for key in ('approach_distance_m', 'lift_distance_m', 'retreat_distance_m'):
            value = payload.get(key)
            if type(value) not in (int, float) or not math.isfinite(value) or value < 0:
                raise ContractError('INVALID_REQUEST', 'Invalid ' + key)
            result[key] = value
        result['object_id'] = payload['object_id']
    return result


def terminal_result(metadata, scene):
    stages = metadata.get('stages', [])
    validations = [s['validation'] for s in stages if isinstance(s, dict) and 'validation' in s]
    if not validations or any(v.get('valid') is not True for v in validations):
        raise ContractError('VALIDATION_FAILED', 'MoveIt trajectory validation missing or failed')
    names = sorted(n for n in metadata.get('start_state', {}) if n.startswith(scene['group'] + '_'))
    if not names:
        raise ContractError('VALIDATION_FAILED', 'No group joint names in plan metadata')
    duration = sum(v['duration_s'] for v in validations)
    waypoints = sum(v['waypoints'] for v in validations)
    if duration <= 0 or waypoints <= 0:
        raise ContractError('VALIDATION_FAILED', 'Trajectory duration or waypoint count is zero')
    return {'plan_id': metadata['plan_id'], 'scene_revision': scene['scene_revision'],
            'robot_model_id': scene['robot_model_id'], 'group': scene['group'],
            'frame': scene['frame_id'], 'tcp_frame': scene['tcp_frame'],
            'planning_time_s': metadata['planning_time_s'],
            'duration_s': duration, 'waypoint_count': waypoints,
            'joint_names': names, 'stages': stages,
            'validation': {'simulated': False, 'joint_limits': 'passed', 'collision': 'passed',
                           'timing': 'passed', 'interpolation_resolution': max(v['resolution'] for v in validations),
                           'collision_samples': sum(v['collision_samples'] for v in validations),
                           'continuous_collision_guarantee': False}}
