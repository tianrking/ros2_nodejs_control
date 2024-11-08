// components/MotionControl/index.jsx
import { memo } from 'react';
import PropTypes from 'prop-types';
import './style.css';

export const MotionControl = memo(({ wheelSpeed, velocity, onWheelSpeedChange, onVelocityChange }) => {
  // 速度限制常量
  const LIMITS = {
    wheel: { min: -2, max: 2 },
    linear: { min: -1, max: 1 },
    angular: { min: -1.57, max: 1.57 }
  };

  const handleWheelInput = (wheel, e, isDelta = false) => {
    let value;
    if (isDelta) {
      e.preventDefault();
      const delta = e.deltaY > 0 ? -0.1 : 0.1;
      value = Math.round((wheelSpeed[wheel] + delta) * 10) / 10;
    } else {
      value = parseFloat(e.target.value) || 0;
    }

    if (value >= LIMITS.wheel.min && value <= LIMITS.wheel.max) {
      onWheelSpeedChange(wheel, value);
    }
  };

  const handleVelocityInput = (type, e, isDelta = false) => {
    let value;
    if (isDelta) {
      e.preventDefault();
      const delta = e.deltaY > 0 ? -0.1 : 0.1;
      value = Math.round((velocity[type] + delta) * 10) / 10;
    } else {
      value = e.target.value === '' ? 0 : parseFloat(e.target.value);
    }

    const limits = LIMITS[type === 'linear' ? 'linear' : 'angular'];
    if (value >= limits.min && value <= limits.max) {
      onVelocityChange(type, value);
    }
  };

  return (
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
                onChange={(e) => handleWheelInput('left', e)}
                onWheel={(e) => handleWheelInput('left', e, true)}
                step="0.1"
                min={LIMITS.wheel.min}
                max={LIMITS.wheel.max}
                className="control-input"
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
                onChange={(e) => handleWheelInput('right', e)}
                onWheel={(e) => handleWheelInput('right', e, true)}
                step="0.1"
                min={LIMITS.wheel.min}
                max={LIMITS.wheel.max}
                className="control-input"
              />
              <span className="unit">m/s</span>
            </div>
          </div>
          <div className="speed-limits">
            <span className="limit-label">速度范围: -2.0 ~ 2.0 m/s</span>
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
                onChange={(e) => handleVelocityInput('linear', e)}
                onWheel={(e) => handleVelocityInput('linear', e, true)}
                step="0.1"
                min={LIMITS.linear.min}
                max={LIMITS.linear.max}
                className="control-input"
              />
              <span className="unit">m/s</span>
            </div>
          </div>
          <div className="control-input-group">
            <label>角速度</label>
            <div className="input-with-unit">
              <input
                type="number"
                value={velocity.angular.toFixed(2)}  // 固定显示两位小数
                onChange={(e) => handleVelocityInput('angular', e)}
                onWheel={(e) => handleVelocityInput('angular', e, true)}
                step="0.1"
                min={LIMITS.angular.min}
                max={LIMITS.angular.max}
                className="control-input"
              />
              <span className="unit">rad/s</span>
            </div>
          </div>
          <div className="speed-limits">
            <span className="limit-label">线速度范围: -1.0 ~ 1.0 m/s</span>
            <span className="limit-label">角速度范围: -1.57 ~ 1.57 rad/s</span>
          </div>
        </div>
      </div>
    </div>
  );
});

MotionControl.propTypes = {
  wheelSpeed: PropTypes.shape({
    left: PropTypes.number.isRequired,
    right: PropTypes.number.isRequired
  }).isRequired,
  velocity: PropTypes.shape({
    linear: PropTypes.number.isRequired,
    angular: PropTypes.number.isRequired
  }).isRequired,
  onWheelSpeedChange: PropTypes.func.isRequired,
  onVelocityChange: PropTypes.func.isRequired
};

MotionControl.displayName = 'MotionControl';