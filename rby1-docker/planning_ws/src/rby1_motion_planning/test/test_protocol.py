import importlib.util
from pathlib import Path
import pytest
spec = importlib.util.spec_from_file_location('protocol', Path(__file__).parents[1] / 'scripts/protocol_core.py')
protocol = importlib.util.module_from_spec(spec)
spec.loader.exec_module(protocol)

def test_common_fixture():
    for line in (Path(__file__).parents[1] / 'fixtures/protocol_v1.ndjson').read_text().splitlines():
        assert protocol.parse_request(line)['v'] == 1

@pytest.mark.parametrize('line,code', [
    ('{"v":true,"id":"x","method":"capabilities"}', 'UNSUPPORTED_VERSION'),
    ('{"v":1,"id":"x","method":"execute"}', 'UNKNOWN_METHOD'),
    ('{"v":1,"id":"x","method":"plan_to_pose","params":{"deadline_s":NaN}}', 'INVALID_JSON'),
    ('{"v":1,"id":"x","method":"plan_to_pose","params":{"deadline_s":0}}', 'INVALID_DEADLINE'),
    ('[]', 'INVALID_REQUEST'), ('{', 'INVALID_JSON'),
])
def test_negative(line, code):
    with pytest.raises(protocol.ProtocolError) as error:
        protocol.parse_request(line)
    assert error.value.code == code

def test_correlation_one_terminal_and_duplicate():
    c = protocol.Correlation()
    req = {'id': 'client-id'}
    c.accept(req, 'internal-id')
    with pytest.raises(protocol.ProtocolError):
        c.accept(req, 'another-id')
    ack = c.response('internal-id', {'type': 'ack', 'task_id': 'internal-id'})
    assert ack == {'id': 'client-id', 'task_id': 'client-id', 'type': 'ack'}
    terminal = c.response('internal-id', {'type': 'event', 'terminal': True})
    assert terminal['id'] == 'client-id'
    assert c.response('internal-id', {'type': 'event', 'terminal': True}) is None
