const rclnodejs = require('rclnodejs');

async function main() {
  // 初始化 ROS2
  await rclnodejs.init();
  console.log('ROS2 initialized successfully');

  // 创建节点
  const node = new rclnodejs.Node('test_node');

  // 订阅所有相关话题
  const topics = [
    '/cmd_vel',
    '/parameter_events',
    '/ping_ping',
    '/rosout',
    '/wheel_left/feedback',
    '/wheel_left/target',
    '/wheel_left/target_speed',
    '/wheel_right/feedback',
    '/wheel_right/target',
    '/wheel_right/target_speed'
  ];

  // 创建一个发布者（用于测试）
  const publisher = node.createPublisher('std_msgs/msg/Float64', '/test_topic');

  // 为每个话题创建订阅者
  topics.forEach(topic => {
    let msgType;
    if (topic === '/cmd_vel') {
      msgType = 'geometry_msgs/msg/Twist';
    } else if (topic === '/parameter_events') {
      msgType = 'rcl_interfaces/msg/ParameterEvent';
    } else if (topic === '/rosout') {
      msgType = 'rcl_interfaces/msg/Log';
    } else {
      msgType = 'std_msgs/msg/Float64';
    }

    node.createSubscription(
      msgType,
      topic,
      (msg) => {
        console.log(`Received message on ${topic}:`, msg);
      }
    );
    console.log(`Subscribed to ${topic}`);
  });

  // 定期发布测试消息
  setInterval(() => {
    publisher.publish({ data: Math.random() * 10 });
    console.log('Published test message');
  }, 1000);

  // 开始 spin
  node.spin();
  console.log('Node is spinning');
}

// 运行主函数
main().catch(console.error);
