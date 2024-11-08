// test_backend.js
import express from 'express';
import { createServer } from 'http';
import { WebSocketServer, WebSocket } from 'ws';
import rclnodejs from 'rclnodejs';

const app = express();
const server = createServer(app);
const wss = new WebSocketServer({ server });

// 存储 ROS2 节点和发布者的全局引用
let ros2Node = null;
let publishers = {};

// 初始化 ROS2
async function initROS2() {
  try {
    console.log('Initializing RCLNode...');
    await rclnodejs.init();
    console.log('RCLNode initialized');

    ros2Node = new rclnodejs.Node('dashboard_node');
    console.log('Node created');

    // 创建发布者
    publishers = {
      '/cmd_vel': ros2Node.createPublisher('geometry_msgs/msg/Twist', '/cmd_vel'),
      '/wheel_left/target': ros2Node.createPublisher('std_msgs/msg/Float64', '/wheel_left/target'),
      '/wheel_right/target': ros2Node.createPublisher('std_msgs/msg/Float64', '/wheel_right/target'),
      '/vehicle_params': ros2Node.createPublisher('std_msgs/msg/String', '/vehicle_params')  // 使用String消息类型来传递JSON数据
    };

    // 创建订阅者
    ros2Node.createSubscription(
      'std_msgs/msg/Float64',
      '/wheel_left/feedback',
      (msg) => {
        broadcastMessage('/wheel_left/feedback', msg.data);
      }
    );

    ros2Node.createSubscription(
      'std_msgs/msg/Float64',
      '/wheel_right/feedback',
      (msg) => {
        broadcastMessage('/wheel_right/feedback', msg.data);
      }
    );

    ros2Node.createSubscription(
      'std_msgs/msg/Float64',
      '/wheel_left/target_speed',
      (msg) => {
        broadcastMessage('/wheel_left/target_speed', msg.data);
      }
    );

    ros2Node.createSubscription(
      'std_msgs/msg/Float64',
      '/wheel_right/target_speed',
      (msg) => {
        broadcastMessage('/wheel_right/target_speed', msg.data);
      }
    );

    // 在创建订阅者的部分添加
    ros2Node.createSubscription(
      'std_msgs/msg/String',
      '/vehicle_params_feedback',  // 假设有一个反馈话题
      (msg) => {
        try {
          const params = JSON.parse(msg.data);
          broadcastMessage('/vehicle_params_feedback', params);
        } catch (error) {
          console.error('Error parsing vehicle params feedback:', error);
        }
      }
    );

    ros2Node.spin();
    console.log('Node is spinning');
    return ros2Node;
  } catch (error) {
    console.error('ROS2 initialization error:', error);
    throw error;
  }
}

// 广播消息给所有客户端
function broadcastMessage(topic, data) {
  wss.clients.forEach(client => {
    if (client.readyState === WebSocket.OPEN) {
      client.send(JSON.stringify({
        topic: topic,
        data: data,
        timestamp: new Date().toISOString()
      }));
    }
  });
}

// WebSocket 连接处理
wss.on('connection', (ws) => {
  console.log('Client connected');
  
  // 处理来自前端的消息
  // 处理来自前端的消息
  ws.on('message', (message) => {
    try {
      const data = JSON.parse(message);
      console.log('Received from client:', data);

      // 处理不同类型的消息
      if (data.type === 'publish') {
        const publisher = publishers[data.topic];
        if (publisher) {
          let msg;
          // 根据话题类型创建相应的消息
          switch(data.topic) {
            case '/cmd_vel':
              msg = {
                linear: { x: data.data.linear, y: 0.0, z: 0.0 },
                angular: { x: 0.0, y: 0.0, z: data.data.angular }
              };
              break;
            case '/vehicle_params':
              // 将参数对象转换为JSON字符串
              msg = {
                data: JSON.stringify({
                  type: data.data.type,
                  params: {
                    wheelRadius: data.data.wheelRadius,
                    vehicleWidth: data.data.vehicleWidth,
                    vehicleLength: data.data.vehicleLength
                  }
                })
              };
              break;
            default:
              msg = { data: data.data };
              break;
          }

          // 发布消息
          publisher.publish(msg);
          console.log('Published to', data.topic, ':', msg);
        } else {
          console.error('Publisher not found for topic:', data.topic);
        }
      }
    } catch (error) {
      console.error('Error processing message:', error);
    }
  });
  
  ws.on('close', () => {
    console.log('Client disconnected');
  });
});

// 启动服务器
const PORT = 3001;
server.listen(PORT, async () => {
  console.log(`Server listening on port ${PORT}`);
  try {
    await initROS2();
    console.log('ROS2 initialization completed');
  } catch (error) {
    console.error('Failed to initialize ROS2:', error);
  }
});

// 优雅关闭
process.on('SIGINT', () => {
  console.log('Shutting down...');
  if (ros2Node) {
    ros2Node.destroy();
  }
  server.close(() => {
    process.exit(0);
  });
});