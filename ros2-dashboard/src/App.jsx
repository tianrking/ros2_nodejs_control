import { useState, useEffect, useRef } from 'react'
import './App.css'

function App() {
  const [connected, setConnected] = useState(false)
  const [error, setError] = useState(null)
  const wsRef = useRef(null)
  const [currentTime, setCurrentTime] = useState(new Date().toLocaleTimeString())
  
  // 实时数据状态
  const [sensorData, setSensorData] = useState({
    wheelLeftFeedback: 0,
    wheelRightFeedback: 0
  })
  
  // 控制参数
  const [wheelSpeed, setWheelSpeed] = useState({
    left: 0,
    right: 0
  })
  const [velocity, setVelocity] = useState({
    linear: 0,
    angular: 0
  })

  // 时间更新效果
  useEffect(() => {
    // 每秒更新时间
    const timer = setInterval(() => {
      setCurrentTime(new Date().toLocaleTimeString())
    }, 1000)

    // 清理函数
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
        
        setSensorData(prev => {
          switch(message.topic) {
            case '/wheel_left/feedback':
              return { ...prev, wheelLeftFeedback: message.data }
            case '/wheel_right/feedback':
              return { ...prev, wheelRightFeedback: message.data }
            default:
              return prev
          }
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
    const speed = parseFloat(value)
    setWheelSpeed(prev => ({
      ...prev,
      [wheel]: speed
    }))
    publishMessage(`/wheel_${wheel}/target`, speed)
  }

  const handleVelocityChange = (type, value) => {
    const vel = parseFloat(value)
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
              <div className="sensor-icon">
                <i className="icon">⚙️</i>
              </div>
              <div className="sensor-info">
                <span className="sensor-label">左轮反馈</span>
                <span className="sensor-value">{sensorData.wheelLeftFeedback.toFixed(2)} m/s</span>
              </div>
            </div>
            <div className="sensor-item">
              <div className="sensor-icon">
                <i className="icon">⚙️</i>
              </div>
              <div className="sensor-info">
                <span className="sensor-label">右轮反馈</span>
                <span className="sensor-value">{sensorData.wheelRightFeedback.toFixed(2)} m/s</span>
              </div>
            </div>
          </div>
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
                    step="0.1"
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
                    step="0.1"
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
                    step="0.1"
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
                    step="0.1"
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