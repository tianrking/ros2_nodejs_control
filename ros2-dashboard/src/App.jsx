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
  
  // 实时数据状态
  const [sensorData, setSensorData] = useState({
    wheelLeftFeedback: 0,
    wheelRightFeedback: 0
  })

  // 历史数据
  const [chartData, setChartData] = useState({
    times: [],
    leftWheel: [],
    rightWheel: []
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
          data: ['左轮速度', '右轮速度'],
          textStyle: {
            color: '#fff'
          }
        },
        toolbox: {
          show: true,
          feature: {
            dataView: { 
              readOnly: true,
              textColor: '#fff',
              backgroundColor: '#1a1a1a',
              borderColor: '#333'
            },
            saveAsImage: {
              backgroundColor: '#1a1a1a'
            }
          },
          iconStyle: {
            borderColor: '#666'
          }
        },
        dataZoom: [
          {
            show: true,
            realtime: true,
            start: 65,
            end: 100,
            textStyle: {
              color: '#fff'
            },
            borderColor: '#333',
            backgroundColor: '#1a1a1a',
            fillerColor: 'rgba(255,255,255,0.1)',
            handleStyle: {
              borderColor: '#666'
            }
          },
          {
            type: 'inside',
            realtime: true,
            start: 65,
            end: 100
          }
        ],
        grid: {
          left: '3%',
          right: '4%',
          bottom: '15%',
          containLabel: true
        },
        xAxis: {
          type: 'category',
          boundaryGap: false,
          data: [],
          axisLine: {
            lineStyle: {
              color: '#666'
            }
          },
          axisLabel: {
            formatter: (value) => new Date(parseInt(value)).toLocaleTimeString(),
            color: '#fff'
          },
          splitLine: {
            show: false
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
          axisLine: {
            lineStyle: {
              color: '#666'
            }
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
            name: '左轮速度',
            type: 'line',
            smooth: true,
            symbol: 'none',
            sampling: 'lttb',
            animation: false,
            lineStyle: {
              width: 2,
              color: '#00ff9d'
            },
            areaStyle: {
              opacity: 0.2,
              color: new echarts.graphic.LinearGradient(0, 0, 0, 1, [
                { offset: 0, color: '#00ff9d' },
                { offset: 1, color: 'rgba(0, 255, 157, 0)' }
              ])
            },
            data: []
          },
          {
            name: '右轮速度',
            type: 'line',
            smooth: true,
            symbol: 'none',
            sampling: 'lttb',
            animation: false,
            lineStyle: {
              width: 2,
              color: '#0091ff'
            },
            areaStyle: {
              opacity: 0.2,
              color: new echarts.graphic.LinearGradient(0, 0, 0, 1, [
                { offset: 0, color: '#0091ff' },
                { offset: 1, color: 'rgba(0, 145, 255, 0)' }
              ])
            },
            data: []
          }
        ]
      };

      chartInstanceRef.current.setOption(option);

      const handleResize = () => {
        chartInstanceRef.current?.resize();
      };

      window.addEventListener('resize', handleResize);

      return () => {
        window.removeEventListener('resize', handleResize);
        chartInstanceRef.current?.dispose();
      };
    }
  }, []);

  // 更新图表数据
  const updateChart = (newData) => {
    if (!chartInstanceRef.current) return;
  
    const { times, leftWheel, rightWheel } = newData;
    
    // 确保数据格式正确
    const formattedData = times.map((time, index) => ({
      time,
      leftWheel: leftWheel[index] || 0,
      rightWheel: rightWheel[index] || 0
    }));
  
    chartInstanceRef.current.setOption({
      xAxis: {
        data: formattedData.map(item => item.time)
      },
      series: [
        {
          name: '左轮速度',
          data: formattedData.map(item => item.leftWheel),
          type: 'line',
          smooth: true,
          showSymbol: false,
          lineStyle: {
            width: 2,
            color: '#00ff9d'  // 绿色
          },
          areaStyle: {
            opacity: 0.2,
            color: new echarts.graphic.LinearGradient(0, 0, 0, 1, [
              { offset: 0, color: '#00ff9d' },
              { offset: 1, color: 'rgba(0, 255, 157, 0)' }
            ])
          }
        },
        {
          name: '右轮速度',
          data: formattedData.map(item => item.rightWheel),
          type: 'line',
          smooth: true,
          showSymbol: false,
          lineStyle: {
            width: 2,
            color: '#0091ff'  // 蓝色
          },
          areaStyle: {
            opacity: 0.2,
            color: new echarts.graphic.LinearGradient(0, 0, 0, 1, [
              { offset: 0, color: '#0091ff' },
              { offset: 1, color: 'rgba(0, 145, 255, 0)' }
            ])
          }
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
        const message = JSON.parse(event.data)
        console.log('Received message:', message)
        
        // 更新当前传感器数据
        setSensorData(prev => {
          const newData = { ...prev }
          if (message.topic === '/wheel_left/feedback') {
            newData.wheelLeftFeedback = message.data
          } else if (message.topic === '/wheel_right/feedback') {
            newData.wheelRightFeedback = message.data
          }
          return newData
        })

        // 更新图表数据
        setChartData(prev => {
          const timestamp = new Date().getTime()
          const newTimes = [...prev.times, timestamp]
          const newLeftWheel = [...prev.leftWheel]
          const newRightWheel = [...prev.rightWheel]

          if (message.topic === '/wheel_left/feedback') {
            newLeftWheel.push(message.data)
            newRightWheel.push(newRightWheel.length > 0 ? newRightWheel[newRightWheel.length - 1] : 0)
          } else if (message.topic === '/wheel_right/feedback') {
            newRightWheel.push(message.data)
            newLeftWheel.push(newLeftWheel.length > 0 ? newLeftWheel[newLeftWheel.length - 1] : 0)
          }

          // 保持数据点数量限制
          const sliceStart = Math.max(0, newTimes.length - MAX_DATA_POINTS)
          const newData = {
            times: newTimes.slice(sliceStart),
            leftWheel: newLeftWheel.slice(sliceStart),
            rightWheel: newRightWheel.slice(sliceStart)
          }

          // 更新图表
          updateChart(newData)

          return newData
        })

      } catch (err) {
        console.error('Error parsing message:', err)
      }
    }

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