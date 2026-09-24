"""Provisional v1 validation. Replace only after supplied spec is available."""
import json
import math

METHODS = frozenset({'capabilities', 'load_scene', 'get_scene', 'reset_scene',
                     'get_state', 'plan_to_pose', 'plan_pick_place', 'preview',
                     'get_task_status', 'cancel', 'check_state', 'cartesian_probe'})
MAX_LINE = 1024 * 1024

class ProtocolError(ValueError):
    def __init__(self, code, message):
        super().__init__(message)
        self.code = code

def parse_request(line):
    if len(line) > MAX_LINE:
        raise ProtocolError('REQUEST_TOO_LARGE', 'Maximum NDJSON line is 1 MiB')
    try:
        req = json.loads(line, parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value)))
    except (ValueError, UnicodeError) as exc:
        raise ProtocolError('INVALID_JSON', str(exc)) from exc
    if not isinstance(req, dict):
        raise ProtocolError('INVALID_REQUEST', 'Request must be an object')
    if type(req.get('v')) is not int or req['v'] != 1:
        raise ProtocolError('UNSUPPORTED_VERSION', 'Expected integer v=1')
    if not isinstance(req.get('id'), str) or not 1 <= len(req['id']) <= 128:
        raise ProtocolError('INVALID_ID', 'id must be a nonempty string of at most 128 characters')
    if not isinstance(req.get('method'), str) or req['method'] not in METHODS:
        raise ProtocolError('UNKNOWN_METHOD', 'Unknown planning-only method')
    if not isinstance(req.get('params', {}), dict):
        raise ProtocolError('INVALID_REQUEST', 'params must be an object')
    value = req.get('params', {}).get('deadline_s', 30)
    if type(value) not in (int, float) or not math.isfinite(value) or not 0 < value <= 120:
        raise ProtocolError('INVALID_DEADLINE', 'deadline_s must be in (0,120]')
    return req

def error_response(request_id, code, message):
    return {'v': 1, 'id': request_id, 'type': 'response', 'ok': False,
            'error': {'code': code, 'message': message}, 'execution_enabled': False}

class Correlation:
    """Track a connection's unique IDs and a single terminal per task."""
    def __init__(self):
        self.seen = set()
        self.pending = {}

    def accept(self, request, wire_id):
        request_id = request['id']
        if request_id in self.seen:
            raise ProtocolError('DUPLICATE_ID', 'Request IDs must be unique per connection')
        if len(self.seen) >= 10000:
            raise ProtocolError('CONNECTION_LIMIT', 'Reconnect after 10000 requests')
        self.seen.add(request_id)
        self.pending[wire_id] = request_id

    def response(self, wire_id, message):
        if wire_id not in self.pending:
            return None
        out = dict(message)
        out['id'] = self.pending[wire_id]
        if out.get('task_id') == wire_id:
            out['task_id'] = out['id']
        if out.get('type') == 'response' or out.get('terminal') is True:
            del self.pending[wire_id]
        return out
