import json
import threading
import websocket_server

# 存储所有连接的客户端
clients = {}

def on_message(client, server, message):
    try:
        data = json.loads(message)
        print(f'Received from client: {data}')

        # 根据消息类型进行处理
        if data['type'] == 'publish':
            broadcast_message(data['topic'], data['data'])
    except Exception as e:
        print(f'Error processing message: {e}')

def on_client_connect(client, server):
    client_id = client['id']
    clients[client_id] = client
    print(f'Client connected: {client_id}')

def on_client_disconnect(client, server):
    client_id = client['id']
    del clients[client_id]
    print(f'Client disconnected: {client_id}')

def broadcast_message(topic, data):
    message = json.dumps({
        'topic': topic,
        'data': data,
        'timestamp': time.strftime('%Y-%m-%dT%H:%M:%S.%fZ', time.gmtime())
    })
    for client in clients.values():
        if client['connected']:
            client['handler'].send(message)

def start_server(host='0.0.0.0', port=3001):
    server = websocket_server.WebsocketServer(host, port)
    server.set_fn_new_client(on_client_connect)
    server.set_fn_client_left(on_client_disconnect)
    server.set_fn_message_received(on_message)

    print(f'WebSocket server started, listening on {host}:{port}')
    server.run_forever()

if __name__ == '__main__':
    start_server()