import { useState, useEffect, useRef } from 'react'
import * as echarts from 'echarts';
import './App.css'

function App() {
  const [connected, setConnected] = useState(false)
  const [error, setError] = useState(null)
  const wsRef = useRef(null)
  const chartRef = useRef(null)
  const chartInstanceRef = useRef(null)
  const [currentTime, setCurrentTime] = useState(new Date().toLocaleTimeString())

  const [paramsModified, setParamsModified] = useState(false)
  const [submitSuccess, setSubmitSuccess] = useState(false)


  // 实时数据状态
  const [sensorData, setSensorData] = useState({
    wheelLeftFeedback: 0,
    wheelRightFeedback: 0
  })

  // 历史数据
  // const [chartData, setChartData] = useState({
  //   times: [],
  //   leftWheel: [],
  //   rightWheel: []
  // })

  const [chartData, setChartData] = useState({
    times: [],
    leftWheelFeedback: [],    // 左轮实际速度
    rightWheelFeedback: [],   // 右轮实际速度
    leftWheelTarget: [],      // 左轮期望速度
    rightWheelTarget: []      // 右轮期望速度
  })

  const MAX_DATA_POINTS = 100 // 最多显示100个数据点

  // 控制参数
  const [wheelSpeed, setWheelSpeed] = useState({
    left: 0,
    right: 0
  })
  const [velocity, setVelocity] = useState({
    linear: 0,
    angular: 0
  })

  const [vehicleType, setVehicleType] = useState('differential')
  const [vehicleParams, setVehicleParams] = useState({
    wheelRadius: 0.033,
    vehicleWidth: 0.160,
    vehicleLength: 0.200
  })

  // 初始化图表
  useEffect(() => {
    if (chartRef.current) {
      chartInstanceRef.current = echarts.init(chartRef.current, 'dark');

      const option = {
        backgroundColor: 'transparent',
        title: {
          text: '实时速度趋势',
          textStyle: {
            fontSize: 14,
            color: '#fff',
            fontWeight: 'normal'
          }
        },
        tooltip: {
          trigger: 'axis',
          axisPointer: {
            type: 'cross',
            label: {
              backgroundColor: '#283b56'
            }
          }
        },
        legend: {
          data: ['左轮实际速度', '右轮实际速度', '左轮期望速度', '右轮期望速度'],
          textStyle: {
            color: '#fff'
          }
        },
        grid: {
          left: '3%',
          right: '4%',
          bottom: '3%',
          containLabel: true
        },
        xAxis: {
          type: 'category',
          boundaryGap: false,
          data: [],
          axisLabel: {
            formatter: (value) => new Date(parseInt(value)).toLocaleTimeString(),
            color: '#fff'
          },
          axisLine: {
            lineStyle: {
              color: '#666'
            }
          }
        },
        yAxis: {
          type: 'value',
          name: '速度 (m/s)',
          nameTextStyle: {
            color: '#fff'
          },
          axisLabel: {
            color: '#fff'
          },
          splitLine: {
            lineStyle: {
              color: 'rgba(255,255,255,0.1)',
              type: 'dashed'
            }
          }
        },
        series: [
          {
            name: '左轮实际速度',
            type: 'line',
            smooth: true,
            symbol: 'none',
            lineStyle: {
              width: 2,
              color: '#00ff9d'  // 绿色
            },
            data: []
          },
          {
            name: '右轮实际速度',
            type: 'line',
            smooth: true,
            symbol: 'none',
            lineStyle: {
              width: 2,
              color: '#0091ff'  // 蓝色
            },
            data: []
          },
          {
            name: '左轮期望速度',
            type: 'line',
            smooth: true,
            symbol: 'none',
            lineStyle: {
              width: 2,
              type: 'dashed',
              color: '#00ff9d'  // 虚线绿色
            },
            data: []
          },
          {
            name: '右轮期望速度',
            type: 'line',
            smooth: true,
            symbol: 'none',
            lineStyle: {
              width: 2,
              type: 'dashed',
              color: '#0091ff'  // 虚线蓝色
            },
            data: []
          }
        ]
      };

      chartInstanceRef.current.setOption(option);

      // 响应窗口变化
      window.addEventListener('resize', () => {
        chartInstanceRef.current?.resize();
      });

      return () => {
        window.removeEventListener('resize', () => {
          chartInstanceRef.current?.resize();
        });
        chartInstanceRef.current?.dispose();
      };
    }
  }, []);

  // 更新图表数据
  // 更新图表数据的函数
  const updateChart = (newData) => {
    if (!chartInstanceRef.current) return;

    chartInstanceRef.current.setOption({
      xAxis: {
        data: newData.times
      },
      series: [
        {
          name: '左轮实际速度',
          data: newData.leftWheelFeedback
        },
        {
          name: '右轮实际速度',
          data: newData.rightWheelFeedback
        },
        {
          name: '左轮期望速度',
          data: newData.leftWheelTarget
        },
        {
          name: '右轮期望速度',
          data: newData.rightWheelTarget
        }
      ]
    });
  };


  // 时间更新效果
  useEffect(() => {
    const timer = setInterval(() => {
      setCurrentTime(new Date().toLocaleTimeString())
    }, 100)

    return () => clearInterval(timer)
  }, [])

  // WebSocket 连接效果
  useEffect(() => {
    const ws = new WebSocket('ws://localhost:3001')
    wsRef.current = ws

    ws.onopen = () => {
      console.log('Connected to WebSocket server')
      setConnected(true)
      setError(null)
    }

    ws.onclose = () => {
      console.log('Disconnected from WebSocket server')
      setConnected(false)
      setError('WebSocket connection closed')
    }

    ws.onerror = (event) => {
      console.error('WebSocket error:', event)
      setError('WebSocket connection error')
    }

    ws.onmessage = (event) => {
      try {
        const message = JSON.parse(event.data);
        console.log('Received message:', message);

        // 更新当前传感器数据
        setSensorData(prev => {
          const newData = { ...prev };
          if (message.topic === '/wheel_left/feedback') {
            newData.wheelLeftFeedback = message.data;
          } else if (message.topic === '/wheel_right/feedback') {
            newData.wheelRightFeedback = message.data;
          }
          return newData;
        });

        // 更新图表数据
        setChartData(prev => {
          const timestamp = new Date().getTime();
          let newData = { ...prev };

          // 如果是新的时间点，添加到时间数组
          if (newData.times[newData.times.length - 1] !== timestamp) {
            newData.times = [...newData.times, timestamp];

            // 对所有数据系列添加新的点，使用最后一个已知值
            newData.leftWheelFeedback = [...newData.leftWheelFeedback,
            newData.leftWheelFeedback[newData.leftWheelFeedback.length - 1] || 0];
            newData.rightWheelFeedback = [...newData.rightWheelFeedback,
            newData.rightWheelFeedback[newData.rightWheelFeedback.length - 1] || 0];
            newData.leftWheelTarget = [...newData.leftWheelTarget,
            newData.leftWheelTarget[newData.leftWheelTarget.length - 1] || 0];
            newData.rightWheelTarget = [...newData.rightWheelTarget,
            newData.rightWheelTarget[newData.rightWheelTarget.length - 1] || 0];
          }

          // 更新最新的数据点
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
          }

          // 保持数据点数量限制
          if (newData.times.length > MAX_DATA_POINTS) {
            const sliceStart = newData.times.length - MAX_DATA_POINTS;
            newData = {
              times: newData.times.slice(sliceStart),
              leftWheelFeedback: newData.leftWheelFeedback.slice(sliceStart),
              rightWheelFeedback: newData.rightWheelFeedback.slice(sliceStart),
              leftWheelTarget: newData.leftWheelTarget.slice(sliceStart),
              rightWheelTarget: newData.rightWheelTarget.slice(sliceStart)
            };
          }

          // 更新图表
          updateChart(newData);

          return newData;
        });

      } catch (err) {
        console.error('Error parsing message:', err);
      }
    };

    return () => {
      ws.close()
    }
  }, [])

  // 发送消息到后端
  const publishMessage = (topic, data) => {
    if (wsRef.current?.readyState === WebSocket.OPEN) {
      wsRef.current.send(JSON.stringify({
        type: 'publish',
        topic: topic,
        data: data
      }))
    }
  }

  const handleWheelSpeedChange = (wheel, value) => {
    const speed = parseFloat(value) || 0
    setWheelSpeed(prev => ({
      ...prev,
      [wheel]: speed
    }))
    publishMessage(`/wheel_${wheel}/target`, speed)
  }

  const handleVelocityChange = (type, value) => {
    const vel = parseFloat(value) || 0
    setVelocity(prev => ({
      ...prev,
      [type]: vel
    }))
    publishMessage('/cmd_vel', {
      linear: type === 'linear' ? vel : velocity.linear,
      angular: type === 'angular' ? vel : velocity.angular
    })
  }

  const handleVehicleTypeChange = (type) => {
    setVehicleType(type);
    // 可以在这里根据不同类型设置默认参数
    switch (type) {
      case 'ackermann':
        setVehicleParams({
          wheelRadius: 0.033,
          vehicleWidth: 0.160,
          vehicleLength: 0.200
        });
        break;
      case 'differential':
        setVehicleParams({
          wheelRadius: 0.033,
          vehicleWidth: 0.160,
          vehicleLength: 0.200
        });
        break;
      // ... 其他类型的默认参数
    }
    setParamsModified(true); // 标记参数已被修改
  };

  // const handleParamChange = (param, value) => {
  //   const numValue = parseFloat(value);
  //   setVehicleParams(prev => {
  //     const newParams = {
  //       ...prev,
  //       [param]: numValue
  //     };
  //     // 发送到后端
  //     publishMessage('/vehicle_params', {
  //       type: vehicleType,
  //       ...newParams
  //     });
  //     return newParams;
  //   });
  // };

  const handleParamChange = (param, value) => {
    const numValue = parseFloat(value);
    setVehicleParams(prev => ({
      ...prev,
      [param]: numValue
    }));
    setParamsModified(true); // 标记参数已被修改
  };

  // 添加参数提交函数
  const handleParamsSubmit = () => {
    // 发送到后端
    publishMessage('/vehicle_params', {
      type: vehicleType,
      ...vehicleParams
    });

    // 显示成功提示
    setSubmitSuccess(true);
    setTimeout(() => setSubmitSuccess(false), 2000); // 2秒后隐藏提示

    // 重置修改标记
    setParamsModified(false);
  };

  return (
    <div className="dashboard">
      {/* 顶部状态栏 */}
      <div className="status-bar">
        <div className="status-item">
          <span className="status-label">系统状态</span>
          <span className={`status-indicator ${connected ? 'connected' : 'disconnected'}`}>
            {connected ? '已连接' : '未连接'}
          </span>
        </div>
        <div className="status-item">
          <span className="status-label">当前时间</span>
          <span className="status-value">{currentTime}</span>
        </div>
      </div>

      {/* 主要内容区 */}
      <div className="main-content">
        {/* 传感器数据卡片 */}
        <div className="dashboard-card">
          <div className="card-header">
            <h2>实时数据监控</h2>
          </div>
          <div className="sensor-grid">
            <div className="sensor-item">
              <div className="sensor-info">
                <span className="sensor-label">左轮反馈</span>
                <span className="sensor-value">{sensorData.wheelLeftFeedback.toFixed(2)} m/s</span>
              </div>
            </div>
            <div className="sensor-item">
              <div className="sensor-info">
                <span className="sensor-label">右轮反馈</span>
                <span className="sensor-value">{sensorData.wheelRightFeedback.toFixed(2)} m/s</span>
              </div>
            </div>
          </div>

          {/* ECharts 图表容器 */}
          <div ref={chartRef} className="chart-container" />
        </div>

        {/* 控制面板卡片 */}
        <div className="dashboard-card">
          <div className="card-header">
            <h2>运动控制</h2>
          </div>
          <div className="control-grid">
            {/* 轮速控制 */}
            <div className="control-section">
              <h3>轮速控制</h3>
              <div className="control-input-group">
                <label>左轮速度</label>
                <div className="input-with-unit">
                  <input
                    type="number"
                    value={wheelSpeed.left}
                    onChange={(e) => handleWheelSpeedChange('left', e.target.value)}
                    onWheel={(e) => {
                      e.preventDefault();
                      const delta = e.deltaY > 0 ? -0.1 : 0.1;
                      const newValue = Math.round((wheelSpeed.left + delta) * 10) / 10;
                      if (newValue >= -2 && newValue <= 2) {
                        handleWheelSpeedChange('left', newValue);
                      }
                    }}
                    step="0.1"
                    min="-2"
                    max="2"
                  />
                  <span className="unit">m/s</span>
                </div>
              </div>
              <div className="control-input-group">
                <label>右轮速度</label>
                <div className="input-with-unit">
                  <input
                    type="number"
                    value={wheelSpeed.right}
                    onChange={(e) => handleWheelSpeedChange('right', e.target.value)}
                    onWheel={(e) => {
                      e.preventDefault();
                      const delta = e.deltaY > 0 ? -0.1 : 0.1;
                      const newValue = Math.round((wheelSpeed.right + delta) * 10) / 10;
                      if (newValue >= -2 && newValue <= 2) {
                        handleWheelSpeedChange('right', newValue);
                      }
                    }}
                    step="0.1"
                    min="-2"
                    max="2"
                  />
                  <span className="unit">m/s</span>
                </div>
              </div>
            </div>

            {/* 速度控制 */}
            <div className="control-section">
              <h3>速度控制</h3>
              <div className="control-input-group">
                <label>线速度</label>
                <div className="input-with-unit">
                  <input
                    type="number"
                    value={velocity.linear}
                    onChange={(e) => handleVelocityChange('linear', e.target.value)}
                    onWheel={(e) => {
                      e.preventDefault();
                      const delta = e.deltaY > 0 ? -0.1 : 0.1;
                      const newValue = Math.round((velocity.linear + delta) * 10) / 10;
                      if (newValue >= -1 && newValue <= 1) {
                        handleVelocityChange('linear', newValue);
                      }
                    }}
                    step="0.1"
                    min="-1"
                    max="1"
                  />
                  <span className="unit">m/s</span>
                </div>
              </div>
              <div className="control-input-group">
                <label>角速度</label>
                <div className="input-with-unit">
                  <input
                    type="number"
                    value={velocity.angular}
                    onChange={(e) => handleVelocityChange('angular', e.target.value)}
                    onWheel={(e) => {
                      e.preventDefault();
                      const delta = e.deltaY > 0 ? -0.1 : 0.1;
                      const newValue = Math.round((velocity.angular + delta) * 10) / 10;
                      if (newValue >= -1.57 && newValue <= 1.57) {
                        handleVelocityChange('angular', newValue);
                      }
                    }}
                    step="0.1"
                    min="-1.57"
                    max="1.57"
                  />
                  <span className="unit">rad/s</span>
                </div>
              </div>
            </div>
          </div>
        </div>

        <div className="dashboard-card">
          <div className="card-header">
            <h2>参数标定</h2>
          </div>
          <div className="control-grid">
            {/* 载具架构选择 */}
            <div className="control-section">
              <h3>载具架构</h3>
              <div className="control-input-group">
                <div className="vehicle-type-select">
                  <select
                    value={vehicleType}
                    onChange={(e) => handleVehicleTypeChange(e.target.value)}
                    className="vehicle-select"
                  >
                    <option value="ackermann">阿克曼小车 (2WD1S)</option>
                    <option value="differential">两轮差速小车-履带车 (2WD)</option>
                    <option value="mecanum">四轮全向车 (4WD)</option>
                    <option value="boat">差速船 (DEV)</option>
                  </select>
                </div>
              </div>
            </div>

            {/* 车辆参数 */}
            <div className="control-section">
              <h3>车辆参数</h3>
              <div className="control-input-group">
                <label>轮子半径 (m)</label>
                <div className="input-with-unit">
                  <input
                    type="number"
                    value={vehicleParams.wheelRadius}
                    onChange={(e) => handleParamChange('wheelRadius', e.target.value)}
                    onWheel={(e) => {
                      e.preventDefault();
                      const delta = e.deltaY > 0 ? -0.001 : 0.001;
                      const newValue = Math.round((vehicleParams.wheelRadius + delta) * 1000) / 1000;
                      if (newValue >= 0.01 && newValue <= 0.5) {
                        handleParamChange('wheelRadius', newValue);
                      }
                    }}
                    step="0.001"
                    min="0.01"
                    max="0.5"
                  />
                  <span className="unit">m</span>
                </div>
              </div>
              <div className="control-input-group">
                <label>车身宽度 (m)</label>
                <div className="input-with-unit">
                  <input
                    type="number"
                    value={vehicleParams.vehicleWidth}
                    onChange={(e) => handleParamChange('vehicleWidth', e.target.value)}
                    onWheel={(e) => {
                      e.preventDefault();
                      const delta = e.deltaY > 0 ? -0.01 : 0.01;
                      const newValue = Math.round((vehicleParams.vehicleWidth + delta) * 100) / 100;
                      if (newValue >= 0.1 && newValue <= 2) {
                        handleParamChange('vehicleWidth', newValue);
                      }
                    }}
                    step="0.01"
                    min="0.1"
                    max="2"
                  />
                  <span className="unit">m</span>
                </div>
              </div>
              <div className="control-input-group">
                <label>车身长度 (m)</label>
                <div className="input-with-unit">
                  <input
                    type="number"
                    value={vehicleParams.vehicleLength}
                    onChange={(e) => handleParamChange('vehicleLength', e.target.value)}
                    onWheel={(e) => {
                      e.preventDefault();
                      const delta = e.deltaY > 0 ? -0.01 : 0.01;
                      const newValue = Math.round((vehicleParams.vehicleLength + delta) * 100) / 100;
                      if (newValue >= 0.1 && newValue <= 2) {
                        handleParamChange('vehicleLength', newValue);
                      }
                    }}
                    step="0.01"
                    min="0.1"
                    max="2"
                  />
                  <span className="unit">m</span>
                </div>
              </div>
              <div className="params-actions">
                <button
                  className={`submit-button ${paramsModified ? 'modified' : ''}`}
                  onClick={handleParamsSubmit}
                  disabled={!paramsModified}
                >
                  应用设置
                </button>
                {submitSuccess && <span className="success-message">设置已更新</span>}
              </div>
            </div>
          </div>
        </div>
      </div>




      {/* 错误提示 */}
      {error && (
        <div className="error-toast">
          {error}
        </div>
      )}
    </div>
  )
}

export default App