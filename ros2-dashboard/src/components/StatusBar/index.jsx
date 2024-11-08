// components/StatusBar/index.jsx
import { memo } from 'react';
import PropTypes from 'prop-types';
import './style.css';

export const StatusBar = memo(({ connected, currentTime }) => {
  return (
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
  );
});

StatusBar.propTypes = {
  connected: PropTypes.bool.isRequired,
  currentTime: PropTypes.string.isRequired
};

StatusBar.displayName = 'StatusBar';