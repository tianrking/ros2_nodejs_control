import { memo, useState } from 'react';
import PropTypes from 'prop-types';
import './style.css';

export const StatusBar = memo(({ 
  connected, 
  currentTime,
  defaultWsUrl,
  onConnect,
  onDisconnect 
}) => {
  const [wsUrl, setWsUrl] = useState(defaultWsUrl);
  const [isEditing, setIsEditing] = useState(false);

  const handleConnect = () => {
    if (connected) {
      onDisconnect();
    } else {
      onConnect(wsUrl);
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter') {
      setIsEditing(false);
      if (!connected) {
        onConnect(wsUrl);
      }
    }
  };

  return (
    <div className="status-bar">
      <div className="status-item ws-connection">
        <span className="status-label">WebSocket服务器</span>
        {isEditing ? (
          <input
            type="text"
            value={wsUrl}
            onChange={(e) => setWsUrl(e.target.value)}
            onBlur={() => setIsEditing(false)}
            onKeyPress={handleKeyPress}
            className="ws-url-input"
            autoFocus
          />
        ) : (
          <span 
            className="ws-url-display"
            onClick={() => setIsEditing(true)}
            title="点击编辑WebSocket地址"
          >
            {wsUrl}
          </span>
        )}
        <div className="connect-section">
          <button 
            className={`connect-button ${connected ? 'connected' : ''}`}
            onClick={handleConnect}
          >
            {connected ? '断开' : '连接'}
          </button>
          <span className={`status-indicator ${connected ? 'connected' : 'disconnected'}`}>
            {connected ? '已连接' : '未连接'}
          </span>
        </div>
      </div>
      <div className="status-item">
        <span className="status-label">当前时间</span>
        <span className="status-value">{currentTime}</span>
      </div>
    </div>
  );
});

StatusBar.propTypes = {
  connected: PropTypes.bool.isRequired,
  currentTime: PropTypes.string.isRequired,
  defaultWsUrl: PropTypes.string.isRequired,
  onConnect: PropTypes.func.isRequired,
  onDisconnect: PropTypes.func.isRequired
};

StatusBar.displayName = 'StatusBar';