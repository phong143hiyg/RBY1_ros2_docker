#!/usr/bin/env python3
import argparse
import json
import socket
import sys
import time

class Client:
    def __init__(self, host='127.0.0.1', port=7447):
        self.socket = socket.create_connection((host, port), timeout=140)
        self.file = self.socket.makefile('rwb')
        self.serial = 0
        self.messages = []

    def send(self, method, params=None, request_id=None):
        self.serial += 1
        request_id = request_id or f'cli-{self.serial}'
        request = {'v': 1, 'id': request_id, 'method': method, 'params': params or {}}
        self.file.write((json.dumps(request, allow_nan=False) + '\n').encode())
        self.file.flush()
        return request_id

    def receive(self):
        line = self.file.readline()
        if not line:
            raise ConnectionError('Service disconnected')
        result = json.loads(line)
        self.messages.append(result)
        return result

    def call(self, method, params=None):
        request_id = self.send(method, params)
        while True:
            response = self.receive()
            if response['id'] == request_id and (response['type'] == 'response' or response.get('terminal')):
                return response

    def close(self):
        self.file.close()
        self.socket.close()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Planning-only NDJSON CLI')
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=7447)
    parser.add_argument('method')
    parser.add_argument('--params', default='{}', help='JSON object or @file.json')
    args = parser.parse_args()
    raw = open(args.params[1:]).read() if args.params.startswith('@') else args.params
    client = Client(args.host, args.port)
    try:
        result = client.call(args.method, json.loads(raw))
        print(json.dumps(result, indent=2))
        sys.exit(0 if result.get('ok') else 1)
    finally:
        client.close()
