#!/usr/bin/env python3
"""Qt contract v1 TCP endpoint; delegates all geometry and planning to MoveIt."""
import asyncio
import json
import time
import uuid
from pathlib import Path

import yaml
from contract_v1 import (MAX_FRAME, ContractError, capabilities, envelope, error,
                         parse_request, plan_params, revision_of, scenario_doc,
                         scene_payload, terminal_result, verify_scene)


class LegacyBackend:
    """One persistent private connection preserves task IDs across Qt reconnects."""
    def __init__(self, host, port):
        self.host, self.port = host, port
        self.writer = None
        self.pending = {}
        self.send_lock = asyncio.Lock()
        self.connect_lock = asyncio.Lock()
        self.disconnect_callback = None

    async def connect(self):
        async with self.connect_lock:
            if self.writer and not self.writer.is_closing():
                return
            reader, self.writer = await asyncio.open_connection(self.host, self.port, limit=MAX_FRAME + 1)
            asyncio.create_task(self.read(reader))

    async def read(self, reader):
        try:
            while line := await reader.readline():
                msg = json.loads(line)
                entry = self.pending.get(msg.get('id'))
                if not entry:
                    continue
                future, callback = entry
                if msg.get('type') == 'event':
                    if callback:
                        await callback(msg)
                    if msg.get('terminal'):
                        self.pending.pop(msg['id'], None)
                elif msg.get('type') in ('ack', 'response'):
                    if not future.done():
                        future.set_result(msg)
                    if msg.get('type') == 'response':
                        self.pending.pop(msg['id'], None)
        except (OSError, ValueError, asyncio.LimitOverrunError):
            pass
        finally:
            if self.writer:
                self.writer.close()
                self.writer = None
            for future, _ in self.pending.values():
                if not future.done():
                    future.set_exception(ConnectionError('MoveIt backend disconnected'))
            self.pending.clear()
            if self.disconnect_callback:
                self.disconnect_callback()

    async def call(self, method, params=None, callback=None, timeout=8, wire_id=None):
        await self.connect()
        ident = wire_id or uuid.uuid4().hex
        future = asyncio.get_running_loop().create_future()
        self.pending[ident] = (future, callback)
        wire = {'v': 1, 'id': ident, 'method': method, 'params': params or {}}
        sent = False
        try:
            async with self.send_lock:
                self.writer.write((json.dumps(wire, separators=(',', ':')) + '\n').encode())
                await self.writer.drain()
                sent = True
            result = await asyncio.wait_for(asyncio.shield(future), timeout)
            if result.get('type') == 'response' and not result.get('ok'):
                issue = result.get('error', {})
                raise ContractError(issue.get('code', 'PLANNING_FAILED'), issue.get('message', 'MoveIt backend error'))
            return result, ident
        except Exception:
            # A timed-out planning ACK can still be followed by a terminal event.
            if not sent or callback is None:
                self.pending.pop(ident, None)
            raise


class ContractServer:
    def __init__(self, backend, fixture_path):
        self.backend = backend
        self.source = yaml.safe_load(Path(fixture_path).read_text())
        self.doc = scenario_doc(self.source, 'pick_place_obstacles')
        self.doc['scenario_id'] = 'pick_place_obstacles'
        self.caps = None
        self.scene = None
        self.tasks = {}
        self.seen = set()
        self.mutation_lock = asyncio.Lock()
        if hasattr(self.backend, 'disconnect_callback'):
            self.backend.disconnect_callback = self.backend_lost

    def backend_lost(self):
        for task in self.tasks.values():
            if task['status'] == 'planning':
                task['backend_lost'] = True

    async def initialize(self):
        # scene_loader and joint_state_broadcaster start independently.
        for attempt in range(40):
            try:
                raw, _ = await self.backend.call('capabilities')
                self.caps = capabilities(raw['result'])
                await self.refresh_scene()
                return
            except (OSError, KeyError, ContractError, TimeoutError, ConnectionError):
                await asyncio.sleep(.5)
        raise RuntimeError('MoveIt worker, scene, or fresh fake joint state unavailable')

    async def refresh_scene(self):
        raw_scene, _ = await self.backend.call('get_scene')
        verify_scene(self.doc, raw_scene['result'])
        state, _ = await self.backend.call('get_state')
        self.scene = scene_payload(self.doc,
            revision_of(raw_scene['result'], state['result']['state']), self.caps['robot_model_id'])

    def task_snapshot(self, task):
        return envelope(task['id'], task['command'], 'event', task['payload'],
                        event_seq=task['seq'], status=task['status'], stage=task['stage'],
                        **({'error': task['error']} if task.get('error') else {}))

    async def write(self, writer, message):
        if writer.is_closing():
            return
        writer.write((json.dumps(message, allow_nan=False, separators=(',', ':')) + '\n').encode())
        await writer.drain()

    async def handle(self, reader, writer):
        try:
            while True:
                line = await reader.readline()
                if not line:
                    return
                try:
                    request = parse_request(line)
                except ContractError as exc:
                    # Framing/JSON errors cannot be correlated reliably. Semantic
                    # errors with an ID receive a normal v1 error response.
                    try:
                        candidate = json.loads(line)
                    except (UnicodeError, ValueError):
                        return
                    if not isinstance(candidate, dict) or not isinstance(candidate.get('request_id'), str) or not candidate['request_id']:
                        return
                    await self.write(writer, envelope(candidate['request_id'],
                        candidate.get('command', '') if isinstance(candidate.get('command'), str) else '',
                        'response', {}, ok=False, error=error(exc.code, str(exc))))
                    continue
                ident, command = request['request_id'], request['command']
                if ident in self.seen:
                    await self.write(writer, envelope(ident, command, 'response', ok=False,
                        error=error('INVALID_REQUEST', 'request_id already used in this server session')))
                    continue
                self.seen.add(ident)
                asyncio.create_task(self.dispatch(request, writer))
        except (OSError, ValueError, asyncio.LimitOverrunError):
            pass
        finally:
            writer.close()
            await writer.wait_closed()

    async def dispatch(self, request, writer):
        ident, command, payload = request['request_id'], request['command'], request['payload']
        try:
            if command == 'get_capabilities':
                result = self.caps
            elif command == 'get_scene':
                if not any(t['status'] == 'planning' for t in self.tasks.values()):
                    await self.refresh_scene()
                result = self.scene
            elif command == 'load_test_scene':
                if any(t['status'] == 'planning' for t in self.tasks.values()):
                    raise ContractError('BUSY', 'Planning worker active')
                scenario = payload.get('scenario_id')
                doc = scenario_doc(self.source, scenario)
                doc['scenario_id'] = scenario
                async with self.mutation_lock:
                    raw, _ = await self.backend.call('load_scene', {'scene': doc})
                    verify_scene(doc, raw['result'])
                    state, _ = await self.backend.call('get_state')
                    self.doc = doc
                    self.scene = scene_payload(doc,
                        revision_of(raw['result'], state['result']['state']), self.caps['robot_model_id'])
                    result = self.scene
            elif command == 'get_task_status':
                task = self.tasks.get(payload.get('target_request_id'))
                if not task or task.get('backend_lost') or time.monotonic() - task['updated'] > self.caps['task_status_ttl_s']:
                    raise ContractError('STATE_UNAVAILABLE', 'Task unknown or status TTL expired')
                result = self.task_snapshot(task)
            elif command == 'cancel_planning':
                task = self.tasks.get(payload.get('target_request_id'))
                if not task or task.get('backend_lost') or time.monotonic() - task['updated'] > self.caps['task_status_ttl_s']:
                    raise ContractError('STATE_UNAVAILABLE', 'Task unknown or status TTL expired')
                if task['status'] == 'planning':
                    task['cancel_requested'] = True
                    if task['accepted']:
                        await self.backend.call('cancel', {'task_id': task['backend_id']})
                result = {}
            elif command == 'preview_plan':
                plan_id = payload.get('plan_id')
                if not isinstance(plan_id, str) or not plan_id:
                    raise ContractError('INVALID_REQUEST', 'plan_id required')
                raw, _ = await self.backend.call('preview', {'plan_id': plan_id})
                result = {'plan_id': plan_id, 'rviz_displayed': False,
                          'display_trajectory_published': bool(raw['result']['published']),
                          'simulated': False, 'execution_enabled': False}
            elif command in ('plan_to_pose', 'plan_pick_place'):
                if any(t['status'] == 'planning' for t in self.tasks.values()):
                    raise ContractError('BUSY', 'Planning worker active')
                await self.refresh_scene()
                if command == 'plan_pick_place' and not self.caps['plan_pick_place_available']:
                    raise ContractError('VALIDATION_FAILED',
                        'Vendor gripper configuration cannot support validated pick/place: '
                        + ', '.join(self.caps['pick_place_errors']))
                params = plan_params(command, payload, self.scene)
                backend_id = uuid.uuid4().hex
                task = {'id': ident, 'command': command, 'seq': 0, 'status': 'planning',
                        'stage': 'accepted', 'payload': {}, 'updated': time.monotonic(),
                        'scene': dict(self.scene), 'backend_id': backend_id,
                        'accepted': False, 'cancel_requested': False}
                self.tasks[ident] = task

                async def terminal(message):
                    if task['status'] != 'planning':
                        return
                    task['seq'] += 1
                    task['updated'] = time.monotonic()
                    if message.get('ok'):
                        try:
                            task['payload'] = terminal_result(message['result'], task['scene'])
                            task['status'] = 'succeeded'
                            task['stage'] = 'complete'
                        except (ContractError, KeyError, TypeError) as exc:
                            task['status'] = 'failed'
                            task['stage'] = 'validation'
                            task['payload'] = {}
                            task['error'] = error('VALIDATION_FAILED', str(exc), task['stage'])
                    else:
                        backend_error = message.get('error', {})
                        task['status'] = 'cancelled' if message.get('state') == 'cancelled' else 'failed'
                        task['stage'] = 'planning'
                        task['payload'] = {}
                        if task['status'] == 'failed':
                            task['error'] = error(backend_error.get('code', 'PLANNING_FAILED'),
                                                  backend_error.get('message', 'Planning failed'), task['stage'])
                    await self.write(writer, self.task_snapshot(task))

                try:
                    raw, _ = await self.backend.call(command, params, callback=terminal, wire_id=backend_id)
                except ContractError:
                    self.tasks.pop(ident, None)  # Backend rejected before ACK.
                    raise
                except (TimeoutError, OSError, ConnectionError):
                    # Acceptance is uncertain. Qt must time out and reconcile
                    # status; a negative ACK would incorrectly unlock planning.
                    return
                if raw.get('type') != 'ack':
                    self.tasks.pop(ident, None)
                    raise ContractError('PLANNING_FAILED', 'Backend did not acknowledge planning')
                task['accepted'] = True
                await self.write(writer, envelope(ident, command, 'response', {}, ok=True))
                if task['cancel_requested'] and task['status'] == 'planning':
                    try:
                        await self.backend.call('cancel', {'task_id': backend_id})
                    except (ContractError, OSError, ConnectionError, TimeoutError):
                        # A failed cancel RPC is not a terminal cancellation.
                        pass
                if task['status'] == 'planning':
                    task['seq'] += 1
                    task['stage'] = 'planning'
                    task['updated'] = time.monotonic()
                    await self.write(writer, self.task_snapshot(task))
                return
            else:
                raise ContractError('UNSUPPORTED_COMMAND', command)
            await self.write(writer, envelope(ident, command, 'response', result, ok=True))
        except ContractError as exc:
            await self.write(writer, envelope(ident, command, 'response', {}, ok=False,
                error=error(exc.code, str(exc))))
        except (OSError, ConnectionError, TimeoutError, KeyError, TypeError, ValueError) as exc:
            await self.write(writer, envelope(ident, command, 'response', {}, ok=False,
                error=error('STATE_UNAVAILABLE', str(exc))))


async def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--bind', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8082)
    parser.add_argument('--backend-port', type=int, default=7447)
    parser.add_argument('--scene-file', required=True)
    args = parser.parse_args()
    service = ContractServer(LegacyBackend('127.0.0.1', args.backend_port), args.scene_file)
    await service.initialize()
    server = await asyncio.start_server(service.handle, args.bind, args.port, limit=MAX_FRAME + 2)
    print(f'PLANNING_CONTRACT_V1_READY {args.bind}:{args.port} execution_enabled=false', flush=True)
    async with server:
        await server.serve_forever()


if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
