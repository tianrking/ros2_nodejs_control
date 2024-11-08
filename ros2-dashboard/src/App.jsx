// App.jsx
import { useState, useEffect, useRef } from 'react';
import { WebSocketClient } from './utils/websocket';
import { StatusBar } from './components/StatusBar';
import { DataMonitor } from './components/DataMonitor';
import { MotionControl } from './components/MotionControl';
import { ParamCalibration } from './components/ParamCalibration';
import './App.css';

// WebSocket 配置
const WS_URL = 'ws://localhost:3001';

// 常量配置
const MAX_DATA_POINTS = 100;
const DEFAULT_VEHICLE_PARAMS = {
  differential: {
    wheelRadius: 0.033,
    vehicleWidth: 0.160,
    vehicleLength: 0.200
  },
  ackermann: {
    wheelRadius: 0.033,
    vehicleWidth: 0.160,
    vehicleLength: 0.200
  },
  mecanum: {
    wheelRadius: 0.030,
    vehicleWidth: 0.180,
    vehicleLength: 0.220
  },
  boat: {
    wheelRadius: 0.040,
    vehicleWidth: 0.200,
    vehicleLength: 0.300
  }
};

function App() {
  // WebSocket 相关状态
  const wsRef = useRef(null);
  const [connected, setConnected] = useState(false);
  const [error, setError] = useState(null);
  
  // 系统时间
  const [currentTime, setCurrentTime] = useState(new Date().toLocaleTimeString());
  
  // 传感器数据状态
  const [sensorData, setSensorData] = useState({
    wheelLeftFeedback: 0,
    wheelRightFeedback: 0
  });

  // 图表数据状态
  const [chartData, setChartData] = useState({
    times: [],
    leftWheelFeedback: [],    // 左轮实际速度
    rightWheelFeedback: [],   // 右轮实际速度
    leftWheelTarget: [],      // 左轮期望速度
    rightWheelTarget: []      // 右轮期望速度
  });

  // 运动控制状态
  const [wheelSpeed, setWheelSpeed] = useState({
    left: 0,
    right: 0
  });
  
  const [velocity, setVelocity] = useState({
    linear: 0,
    angular: 0
  });

  // 参数标定状态
  const [vehicleType, setVehicleType] = useState('differential');
  const [vehicleParams, setVehicleParams] = useState(DEFAULT_VEHICLE_PARAMS.differential);
  const [paramsModified, setParamsModified] = useState(false);
  const [submitSuccess, setSubmitSuccess] = useState(false);

  // 更新图表数据的函数
  const updateChart = (newData) => {
    setChartData(newData);
  };

  // WebSocket 消息处理
  const handleWebSocketMessage = (message) => {
    try {
      console.log('Received message:', message);
      
      // 更新传感器数据
      if (message.topic === '/wheel_left/feedback' || message.topic === '/wheel_right/feedback') {
        setSensorData(prev => ({
          ...prev,
          [message.topic === '/wheel_left/feedback' ? 'wheelLeftFeedback' : 'wheelRightFeedback']: message.data
        }));
      }

      // 更新图表数据
      const timestamp = new Date().getTime();
      setChartData(prev => {
        const newData = { ...prev };
        
        // 如果是新的时间点，添加到时间数组
        if (prev.times[prev.times.length - 1] !== timestamp) {
          newData.times = [...prev.times, timestamp];
          
          // 为所有数据系列添加新点，使用最近的已知值
          ['leftWheelFeedback', 'rightWheelFeedback', 'leftWheelTarget', 'rightWheelTarget'].forEach(series => {
            newData[series] = [...prev[series], prev[series][prev[series].length - 1] || 0];
          });
        }

        // 更新最新数据点
        const lastIndex = newData.times.length - 1;
        switch (message.topic) {
          case '/wheel_left/feedback':
            newData.leftWheelFeedback[lastIndex] = message.data;
            break;
          case '/wheel_right/feedback':
            newData.rightWheelFeedback[lastIndex] = message.data;
            break;
          case '/wheel_left/target_speed':
            newData.leftWheelTarget[lastIndex] = message.data;
            break;
          case '/wheel_right/target_speed':
            newData.rightWheelTarget[lastIndex] = message.data;
            break;
          default:
            break;
        }

        // 保持数据点数量限制
        if (newData.times.length > MAX_DATA_POINTS) {
          const sliceStart = newData.times.length - MAX_DATA_POINTS;
          Object.keys(newData).forEach(key => {
            newData[key] = newData[key].slice(sliceStart);
          });
        }

        return newData;
      });
    } catch (err) {
      console.error('Error processing message:', err);
    }
  };// WebSocket 连接效果
  useEffect(() => {
    wsRef.current = new WebSocketClient(WS_URL, {
      onOpen: () => {
        console.log('WebSocket connected');
        setConnected(true);
        setError(null);
      },
      onClose: () => {
        console.log('WebSocket disconnected');
        setConnected(false);
        setError('WebSocket connection closed');
      },
      onError: (error) => {
        console.error('WebSocket error:', error);
        setError('WebSocket connection error');
      },
      onMessage: handleWebSocketMessage
    });

    wsRef.current.connect();

    return () => {
      wsRef.current?.close();
    };
  }, []);

  // 系统时间更新效果
  useEffect(() => {
    const timer = setInterval(() => {
      setCurrentTime(new Date().toLocaleTimeString());
    }, 100);

    return () => clearInterval(timer);
  }, []);

  // 发送消息到后端
  const publishMessage = (topic, data) => {
    if (wsRef.current) {
      wsRef.current.send({
        type: 'publish',
        topic: topic,
        data: data
      });
    }
  };

  // 轮速控制处理函数
  const handleWheelSpeedChange = (wheel, value) => {
    const speed = parseFloat(value) || 0;
    setWheelSpeed(prev => ({
      ...prev,
      [wheel]: speed
    }));
    publishMessage(`/wheel_${wheel}/target`, speed);
  };

  // 速度控制处理函数
  const handleVelocityChange = (type, value) => {
    const vel = parseFloat(value) || 0;
    setVelocity(prev => ({
      ...prev,
      [type]: vel
    }));
    publishMessage('/cmd_vel', {
      linear: type === 'linear' ? vel : velocity.linear,
      angular: type === 'angular' ? vel : velocity.angular
    });
  };

  // 在现有的状态定义部分添加 PID 状态
  const [pidParams, setPidParams] = useState({
    p: 1.0,
    i: 0.1,
    d: 0.01
  });

  // PID 参数是否被修改的状态
  const [pidModified, setPidModified] = useState(false);
  const [pidSubmitSuccess, setPidSubmitSuccess] = useState(false);

  // 在处理函数部分添加
  const handlePidParamChange = (param, value) => {
    setPidParams(prev => ({
      ...prev,
      [param]: value
    }));
    setPidModified(true);
  };

  const handlePidParamsSubmit = () => {
    // 发送 PID 参数到后端
    publishMessage('/pid_params', {
      p: pidParams.p,
      i: pidParams.i,
      d: pidParams.d
    });

    // 显示成功提示
    setPidSubmitSuccess(true);
    setTimeout(() => {
      setPidSubmitSuccess(false);
    }, 2000);

    // 重置修改标记
    setPidModified(false);
  };

  // 车辆类型变更处理函数
  const handleVehicleTypeChange = (type) => {
    setVehicleType(type);
    setVehicleParams(DEFAULT_VEHICLE_PARAMS[type]);
    setParamsModified(true);
  };

  // 参数变更处理函数
  const handleParamChange = (param, value) => {
    setVehicleParams(prev => ({
      ...prev,
      [param]: value
    }));
    setParamsModified(true);
  };

  // 参数提交处理函数
  const handleParamsSubmit = () => {
    publishMessage('/vehicle_params', {
      type: vehicleType,
      ...vehicleParams
    });

    // 显示成功提示
    setSubmitSuccess(true);
    setTimeout(() => {
      setSubmitSuccess(false);
    }, 2000);

    // 重置修改标记
    setParamsModified(false);
  };

  return (
    <div className="dashboard">
      <StatusBar 
        connected={connected} 
        currentTime={currentTime} 
      />
      
      <div className="main-content">
        <DataMonitor 
          sensorData={sensorData} 
          chartData={chartData}
        />

        <MotionControl 
          wheelSpeed={wheelSpeed}
          velocity={velocity}
          pidParams={pidParams}
          onWheelSpeedChange={handleWheelSpeedChange}
          onVelocityChange={handleVelocityChange}
          onPidParamChange={handlePidParamChange}
          onPidParamsSubmit={handlePidParamsSubmit}
        />

        <ParamCalibration 
          vehicleType={vehicleType}
          vehicleParams={vehicleParams}
          paramsModified={paramsModified}
          submitSuccess={submitSuccess}
          onVehicleTypeChange={handleVehicleTypeChange}
          onParamChange={handleParamChange}
          onSubmit={handleParamsSubmit}
        />
      </div>

      {error && (
        <div className="error-toast">
          {error}
        </div>
      )}
    </div>
  );
}

export default App;