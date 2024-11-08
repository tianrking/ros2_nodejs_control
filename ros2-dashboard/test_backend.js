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
      '/vehicle_params': ros2Node.createPublisher('std_msgs/msg/String', '/vehicle_params'),
      // 添加 PID 参数发布者
      '/pid_params': ros2Node.createPublisher('std_msgs/msg/String', '/pid_params'),
      '/pid_params_left': ros2Node.createPublisher('std_msgs/msg/String', '/pid_params_left'),
      '/pid_params_right': ros2Node.createPublisher('std_msgs/msg/String', '/pid_params_right')
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

    ros2Node.createSubscription(
      'std_msgs/msg/String',
      '/vehicle_params_feedback',
      (msg) => {
        try {
          const params = JSON.parse(msg.data);
          broadcastMessage('/vehicle_params_feedback', params);
        } catch (error) {
          console.error('Error parsing vehicle params feedback:', error);
        }
      }
    );

    // 在后端的 initROS2 函数中添加新的订阅者
    ros2Node.createSubscription(
      'sensor_msgs/msg/NavSatFix',  // GPS消息类型
      '/gps/fix',                   // GPS话题
      (msg) => {
        broadcastMessage('/gps/fix', {
          latitude: msg.latitude,
          longitude: msg.longitude,
          altitude: msg.altitude
        });
      }
    );

    ros2Node.createSubscription(
      'std_msgs/msg/Float32',      // 航向角消息类型
      '/heading',                  // 航向角话题
      (msg) => {
        broadcastMessage('/heading', msg.data);
      }
    );

    // 添加 PID 参数反馈订阅
    ros2Node.createSubscription(
      'std_msgs/msg/String',
      '/pid_params_feedback',
      (msg) => {
        try {
          const pidParams = JSON.parse(msg.data);
          broadcastMessage('/pid_params_feedback', pidParams);
        } catch (error) {
          console.error('Error parsing PID params feedback:', error);
        }
      }
    );

    // 可以分别订阅左右轮的 PID 参数反馈
    ros2Node.createSubscription(
      'std_msgs/msg/String',
      '/pid_params_left_feedback',
      (msg) => {
        try {
          const pidParams = JSON.parse(msg.data);
          broadcastMessage('/pid_params_left_feedback', pidParams);
        } catch (error) {
          console.error('Error parsing left wheel PID params feedback:', error);
        }
      }
    );

    ros2Node.createSubscription(
      'std_msgs/msg/String',
      '/pid_params_right_feedback',
      (msg) => {
        try {
          const pidParams = JSON.parse(msg.data);
          broadcastMessage('/pid_params_right_feedback', pidParams);
        } catch (error) {
          console.error('Error parsing right wheel PID params feedback:', error);
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
            case '/pid_params':
              msg = {
                data: JSON.stringify({
                  p: data.data.p,
                  i: data.data.i,
                  d: data.data.d
                })
              };
              break;
            case '/pid_params_left':
              msg = {
                data: JSON.stringify({
                  p: data.data.p,
                  i: data.data.i,
                  d: data.data.d,
                  wheel: 'left'
                })
              };
              break;
            case '/pid_params_right':
              msg = {
                data: JSON.stringify({
                  p: data.data.p,
                  i: data.data.i,
                  d: data.data.d,
                  wheel: 'right'
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