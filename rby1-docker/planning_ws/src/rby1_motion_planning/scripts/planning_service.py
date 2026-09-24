#!/usr/bin/env python3
"""TCP NDJSON transport; ROS retains plans and owns task completion."""
import asyncio
import json
import threading
import uuid
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String
from protocol_core import Correlation, MAX_LINE, ProtocolError, error_response, parse_request

class Service(Node):
    def __init__(self, loop):
        super().__init__('planning_ndjson_service')
        self.loop = loop
        self.declare_parameter('bind', '127.0.0.1')
        self.declare_parameter('port', 7447)
        self.publisher = self.create_publisher(String, '/rby1_planning/requests', 100)
        self.subscription = self.create_subscription(String, '/rby1_planning/responses', self.on_response, 100)
        self.routes = {}

    def on_response(self, message):
        try:
            response = json.loads(message.data)
        except ValueError:
            return
        self.loop.call_soon_threadsafe(self.deliver, response)

    def deliver(self, response):
        route = self.routes.get(response.get('id'))
        if route:
            queue, correlation, prefix = route
            out = correlation.response(response['id'], response)
            if out:
                # Translate referenced task IDs in status/cancel responses for this connection.
                if isinstance(out.get('result'), dict):
                    out['result'] = self.translate(out['result'], prefix)
                queue.put_nowait(out)
            if response.get('type') == 'response' or response.get('terminal') is True:
                self.routes.pop(response['id'], None)

    def translate(self, value, prefix):
        if isinstance(value, dict):
            return {key: (item[len(prefix):] if key in ('id', 'task_id') and isinstance(item, str) and item.startswith(prefix)
                          else self.translate(item, prefix)) for key, item in value.items()}
        if isinstance(value, list):
            return [self.translate(item, prefix) for item in value]
        return value

    async def handle_client(self, reader, writer):
        prefix = uuid.uuid4().hex + ':'
        correlation = Correlation()
        queue = asyncio.Queue(maxsize=1000)
        async def send():
            while True:
                response = await queue.get()
                writer.write((json.dumps(response, allow_nan=False, separators=(',', ':')) + '\n').encode())
                await writer.drain()
        sender = asyncio.create_task(send())
        try:
            while True:
                line = await reader.readline()
                if not line:
                    break
                request_id = ''
                try:
                    # Recover ID for correlated protocol errors where JSON is parseable.
                    try:
                        candidate = json.loads(line)
                        if isinstance(candidate, dict) and isinstance(candidate.get('id'), str):
                            request_id = candidate['id']
                    except ValueError:
                        pass
                    request = parse_request(line)
                    wire_id = prefix + request['id']
                    correlation.accept(request, wire_id)
                    if self.publisher.get_subscription_count() == 0:
                        await queue.put(error_response(request['id'], 'BACKEND_UNAVAILABLE', 'ROS worker is not ready'))
                        correlation.pending.pop(wire_id, None)
                        continue
                    self.routes[wire_id] = (queue, correlation, prefix)
                    outgoing = dict(request, id=wire_id)
                    params = dict(request.get('params', {}))
                    if request['method'] in ('cancel', 'get_task_status'):
                        params['task_id'] = prefix + str(params.get('task_id', ''))
                    outgoing['params'] = params
                    self.publisher.publish(String(data=json.dumps(outgoing, allow_nan=False)))
                except ProtocolError as exc:
                    await queue.put(error_response(request_id, exc.code, str(exc)))
                if queue.qsize() > 900:
                    break
        except (ConnectionError, ValueError, asyncio.LimitOverrunError):
            pass
        finally:
            # Disconnect requests cancellation; worker remains active until it has actually stopped.
            for wire_id in list(correlation.pending):
                self.routes.pop(wire_id, None)
                self.publisher.publish(String(data=json.dumps({'v': 1, 'id': prefix + 'disconnect-' + uuid.uuid4().hex,
                    'method': 'cancel', 'params': {'task_id': wire_id}})))
            sender.cancel()
            await asyncio.gather(sender, return_exceptions=True)
            writer.close()
            await writer.wait_closed()

async def serve():
    rclpy.init()
    service = Service(asyncio.get_running_loop())
    def spin_ros():
        try:
            rclpy.spin(service)
        except ExternalShutdownException:
            pass
    thread = threading.Thread(target=spin_ros, daemon=True)
    thread.start()
    try:
        host = service.get_parameter('bind').value
        port = service.get_parameter('port').value
        server = await asyncio.start_server(service.handle_client, host, port, limit=MAX_LINE + 1)
        service.get_logger().info(f'NDJSON_READY {host}:{port} execution_enabled=false schema=internal-backend-v1')
        async with server:
            await server.serve_forever()
    finally:
        rclpy.try_shutdown()
        thread.join(timeout=5)
        service.destroy_node()

if __name__ == '__main__':
    try:
        asyncio.run(serve())
    except (KeyboardInterrupt, asyncio.CancelledError):
        pass
