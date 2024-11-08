// App.jsx
import { useState, useEffect } from 'react'
import './App.css'

function App() {
  const [count, setCount] = useState(0)
  const [connected, setConnected] = useState(false)
  const [messages, setMessages] = useState([])
  const [error, setError] = useState(null)

  useEffect(() => {
    // 创建 WebSocket 连接
    const ws = new WebSocket('ws://localhost:3001')
    
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
        
        setMessages(prev => [
          ...prev, 
          {
            time: new Date(message.timestamp).toLocaleTimeString(),
            topic: message.topic,
            value: message.data
          }
        ].slice(-10)) // 只保留最近10条消息
      } catch (err) {
        console.error('Error parsing message:', err)
      }
    }

    // 清理函数
    return () => {
      ws.close()
    }
  }, []) // 空依赖数组意味着只在组件挂载时运行一次

  // 格式化数值
  const formatValue = (value) => {
    return typeof value === 'number' ? value.toFixed(2) : value
  }

  return (
    <div className="app-container">
      <div className="header">
        <h1>ROS2 Dashboard</h1>
        <div className={`connection-status ${connected ? 'connected' : 'disconnected'}`}>
          Status: {connected ? 'Connected' : 'Disconnected'}
        </div>
      </div>

      <div className="content">
        <div className="box">
          <h2>Sensor Data</h2>
          {error && (
            <div className="error-message">
              Error: {error}
            </div>
          )}
          {messages.length > 0 ? (
            <div className="message-list">
              {messages.map((msg, index) => (
                <div key={index} className="message-item">
                  <span className="message-time">{msg.time}</span>
                  <span className="message-topic">{msg.topic}</span>
                  <span className="message-value">{formatValue(msg.value)}</span>
                </div>
              ))}
            </div>
          ) : (
            <div className="no-data">
              No data available
              {connected && ' - Waiting for messages...'}
            </div>
          )}
        </div>

        <div className="box">
          <h2>Robot Control</h2>
          <div className="control-panel">
            <div className="status-info">
              <p>Connection Status: {connected ? 'Online' : 'Offline'}</p>
              <p>Messages Received: {messages.length}</p>
            </div>
            <button 
              className="control-button"
              onClick={() => setCount(count + 1)}
            >
              Test Button ({count})
            </button>
          </div>
        </div>
      </div>
    </div>
  )
}

export default App