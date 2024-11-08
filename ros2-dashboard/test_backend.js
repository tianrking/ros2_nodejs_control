// test_backend.js
import express from 'express';
import { createServer } from 'http';
import { WebSocketServer, WebSocket } from 'ws';
import rclnodejs from 'rclnodejs';

const app = express();
const server = createServer(app);
const wss = new WebSocketServer({ server });

// 初始化 ROS2
async function initROS2() {
  try {
    console.log('Initializing RCLNode...');
    // 确保 ROS2 环境已经设置
    await rclnodejs.init();
    console.log('RCLNode initialized');

    const node = new rclnodejs.Node('dashboard_node');
    console.log('Node created');

    // 创建订阅者
    const subscription = node.createSubscription(
      'std_msgs/msg/Float64',
      '/wheel_left/feedback',
      (msg) => {
        console.log('Received message:', msg.data);
        // 广播消息到所有连接的客户端
        wss.clients.forEach(client => {
          if (client.readyState === WebSocket.OPEN) {
            client.send(JSON.stringify({
              topic: '/wheel_left/feedback',
              data: msg.data,
              timestamp: new Date().toISOString()
            }));
          }
        });
      }
    );

    // 创建测试发布者
    const publisher = node.createPublisher('std_msgs/msg/Float64', '/test_topic');

    // 每秒发布一次测试数据
    setInterval(() => {
      const msg = {
        data: Math.random() * 10
      };
      publisher.publish(msg);
      console.log('Published:', msg.data);
    }, 1000);

    // 启动节点
    node.spin();
    console.log('Node is spinning');

    return node;
  } catch (error) {
    console.error('ROS2 initialization error:', error);
    throw error;
  }
}

// WebSocket 连接处理
wss.on('connection', (ws) => {
  console.log('Client connected');
  
  ws.on('message', (message) => {
    console.log('Received from client:', message.toString());
  });
  
  ws.on('close', () => {
    console.log('Client disconnected');
  });
});

// HTTP 路由
app.get('/health', (req, res) => {
  res.json({ status: 'ok' });
});

// 启动服务器
const PORT = 3001;
server.listen(PORT, async () => {
  console.log(`Server listening on port ${PORT}`);
  try {
    await initROS2();
    console.log('ROS2 initialization completed');
  } catch (error) {
    console.error('Failed to initialize ROS2:', error.message);
  }
});

// 优雅关闭
process.on('SIGINT', () => {
  console.log('Shutting down...');
  server.close(() => {
    process.exit(0);
  });
});