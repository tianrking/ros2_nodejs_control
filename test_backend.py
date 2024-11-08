import json
import threading
import time
import websocket_server
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, String, Float32
from sensor_msgs.msg import NavSatFix

# 存储所有连接的客户端
clients = {}

# ROS2 节点和发布者、订阅者
ros2_node = None
publishers = {}
subscribers = {}

def on_message(client, server, message):
    try:
        data = json.loads(message)
        print(f'Received from client: {data}')

        # 根据消息类型进行处理
        if data['type'] == 'publish':
            publish_ros2_message(data['topic'], data['data'])
        elif data['type'] == 'subscribe':
            subscribe_to_ros2_topic(data['topic'])
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
        if 'connected' in client and client['connected']:
            client['handler'].send(message)

def publish_ros2_message(topic, data):
    publisher = publishers.get(topic)
    if publisher:
        msg = None
        if topic == '/cmd_vel':
            msg = Twist()
            msg.linear.x = data['linear']
            msg.angular.z = data['angular']
        elif topic == '/wheel_left/target':
            msg = Float64()
            msg.data = data
        elif topic == '/wheel_right/target':
            msg = Float64()
            msg.data = data
        elif topic == '/vehicle_params':
            msg = String()
            msg.data = json.dumps({
                'type': data['type'],
                'params': {
                    'wheelRadius': data['wheelRadius'],
                    'vehicleWidth': data['vehicleWidth'],
                    'vehicleLength': data['vehicleLength']
                }
            })
        elif topic == '/pid_params':
            msg = String()
            msg.data = json.dumps({
                'p': data['p'],
                'i': data['i'],
                'd': data['d']
            })
        elif topic == '/pid_params_left':
            msg = String()
            msg.data = json.dumps({
                'p': data['p'],
                'i': data['i'],
                'd': data['d'],
                'wheel': 'left'
            })
        elif topic == '/pid_params_right':
            msg = String()
            msg.data = json.dumps({
                'p': data['p'],
                'i': data['i'],
                'd': data['d'],
                'wheel': 'right'
            })

        if msg:
            publisher.publish(msg)
            print(f'Published to {topic}: {msg}')
    else:
        print(f'Publisher not found for topic: {topic}')

def subscribe_to_ros2_topic(topic):
    # 订阅者已经在 init_ros2 中创建完毕, 不需要再次创建
    print(f'Subscribed to topic: {topic}')

def init_ros2():
    global ros2_node, publishers, subscribers

    rclpy.init()
    ros2_node = Node('dashboard_node')
    print('ROS2 node created')

    # 创建发布者
    publishers = {
        '/cmd_vel': ros2_node.create_publisher(Twist, '/cmd_vel', 10),
        '/wheel_left/target': ros2_node.create_publisher(Float64, '/wheel_left/target', 10),
        '/wheel_right/target': ros2_node.create_publisher(Float64, '/wheel_right/target', 10),
        '/vehicle_params': ros2_node.create_publisher(String, '/vehicle_params', 10),
        '/pid_params': ros2_node.create_publisher(String, '/pid_params', 10),
        '/pid_params_left': ros2_node.create_publisher(String, '/pid_params_left', 10),
        '/pid_params_right': ros2_node.create_publisher(String, '/pid_params_right', 10)
    }

    # 创建订阅者
    subscribers = {
        '/wheel_left/feedback': ros2_node.create_subscription(
            Float64, '/wheel_left/feedback', lambda msg: on_ros2_message('/wheel_left/feedback', msg.data), 10
        ),
        '/wheel_right/feedback': ros2_node.create_subscription(
            Float64, '/wheel_right/feedback', lambda msg: on_ros2_message('/wheel_right/feedback', msg.data), 10
        ),
        '/wheel_left/target_speed': ros2_node.create_subscription(
            Float64, '/wheel_left/target_speed', lambda msg: on_ros2_message('/wheel_left/target_speed', msg.data), 10
        ),
        '/wheel_right/target_speed': ros2_node.create_subscription(
            Float64, '/wheel_right/target_speed', lambda msg: on_ros2_message('/wheel_right/target_speed', msg.data), 10
        ),
        '/vehicle_params_feedback': ros2_node.create_subscription(
            String, '/vehicle_params_feedback', lambda msg: on_ros2_message('/vehicle_params_feedback', json.loads(msg.data)), 10
        ),
        '/gps/fix': ros2_node.create_subscription(
            NavSatFix, '/gps/fix', lambda msg: on_ros2_message('/gps/fix', {
                'latitude': msg.latitude,
                'longitude': msg.longitude,
                'altitude': msg.altitude
            }), 10
        ),
        '/heading': ros2_node.create_subscription(
            Float32, '/heading', lambda msg: on_ros2_message('/heading', msg.data), 10
        ),
        '/pid_params_feedback': ros2_node.create_subscription(
            String, '/pid_params_feedback', lambda msg: on_ros2_message('/pid_params_feedback', json.loads(msg.data)), 10
        ),
        '/pid_params_left_feedback': ros2_node.create_subscription(
            String, '/pid_params_left_feedback', lambda msg: on_ros2_message('/pid_params_left_feedback', json.loads(msg.data)), 10
        ),
        '/pid_params_right_feedback': ros2_node.create_subscription(
            String, '/pid_params_right_feedback', lambda msg: on_ros2_message('/pid_params_right_feedback', json.loads(msg.data)), 10
        )
    }

    print('Subscribers created:')
    for topic, subscriber in subscribers.items():
        print(f'- {topic}')

    spin_thread = threading.Thread(target=rclpy.spin, args=(ros2_node,))
    spin_thread.start()
    print('ROS2 node is spinning')

def on_ros2_message(topic, data):
    print(f'Received ROS2 message on topic {topic}: {data}')
    broadcast_message(topic, data)

def broadcast_message(topic, data):
    message = json.dumps({
        'topic': topic,
        'data': data,
        'timestamp': time.strftime('%Y-%m-%dT%H:%M:%S.%fZ', time.gmtime())
    })
    for client_id, client in clients.items():
        if 'connected' in client and client['connected']:
            try:
                client['handler'].send(message)
            except Exception as e:
                print(f'Error sending message to client {client_id}: {e}')
                del clients[client_id]
                print(f'Client {client_id} disconnected')

def start_server(host='0.0.0.0', port=3001):
    server = websocket_server.WebsocketServer(host, port)
    server.set_fn_new_client(on_client_connect)
    server.set_fn_client_left(on_client_disconnect)
    server.set_fn_message_received(on_message)

    print(f'WebSocket server started, listening on {host}:{port}')
    server.run_forever()

if __name__ == '__main__':
    try:
        init_ros2()
    except Exception as e:
        print(f'Failed to initialize ROS2: {e}')

    start_server()