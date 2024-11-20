// components/MotionControl/index.jsx
import { memo } from 'react';
import PropTypes from 'prop-types';
import './style.css';

export const MotionControl = memo(({ 
  wheelSpeed, 
  velocity, 
  pidParams,
  onWheelSpeedChange, 
  onVelocityChange,
  onPidParamChange,
  onPidParamsSubmit
}) => {
  // 速度限制常量
  const LIMITS = {
    wheel: { min: -1000, max: 1000 },
    linear: { min: -20, max: 20 },
    angular: { min: -180, max: 180 },
    pid: {
      p: { min: 0, max: 100, step: 0.1 },
      i: { min: 0, max: 100, step: 0.1 },
      d: { min: 0, max: 100, step: 0.01 }
    }
  };

  const handleWheelInput = (wheel, e, isDelta = false) => {
    let value;
    if (isDelta) {
      e.preventDefault();
      const delta = e.deltaY > 0 ? -0.1 : 0.1;
      value = Math.round((wheelSpeed[wheel] + delta) * 10) / 10;
    } else {
      value = e.target.value === '' ? 0 : parseFloat(e.target.value);
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

    const limits = type === 'linear' ? LIMITS.linear : LIMITS.angular;
    if (value >= limits.min && value <= limits.max) {
      onVelocityChange(type, value);
    }
  };

  const handlePidInput = (param, e, isDelta = false) => {
    let value;
    if (isDelta) {
      e.preventDefault();
      const delta = e.deltaY > 0 ? -LIMITS.pid[param].step : LIMITS.pid[param].step;
      value = Math.round((pidParams[param] + delta) * 100) / 100;
    } else {
      value = parseFloat(e.target.value) || 0;
    }

    if (value >= LIMITS.pid[param].min && value <= LIMITS.pid[param].max) {
      onPidParamChange(param, value);
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
                value={velocity.angular.toFixed(2)}
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

        {/* PID参数控制 */}
        <div className="control-section pid-section">
          <h3>PID 参数设定</h3>
          <div className="control-input-group">
            <label>比例系数 (P)</label>
            <div className="input-with-unit">
              <input
                type="number"
                value={pidParams.p.toFixed(3)}
                onChange={(e) => handlePidInput('p', e)}
                onWheel={(e) => handlePidInput('p', e, true)}
                step={LIMITS.pid.p.step}
                min={LIMITS.pid.p.min}
                max={LIMITS.pid.p.max}
                className="control-input pid-input"
              />
            </div>
          </div>
          <div className="control-input-group">
            <label>积分系数 (I)</label>
            <div className="input-with-unit">
              <input
                type="number"
                value={pidParams.i.toFixed(3)}
                onChange={(e) => handlePidInput('i', e)}
                onWheel={(e) => handlePidInput('i', e, true)}
                step={LIMITS.pid.i.step}
                min={LIMITS.pid.i.min}
                max={LIMITS.pid.i.max}
                className="control-input pid-input"
              />
            </div>
          </div>
          <div className="control-input-group">
            <label>微分系数 (D)</label>
            <div className="input-with-unit">
              <input
                type="number"
                value={pidParams.d.toFixed(3)}
                onChange={(e) => handlePidInput('d', e)}
                onWheel={(e) => handlePidInput('d', e, true)}
                step={LIMITS.pid.d.step}
                min={LIMITS.pid.d.min}
                max={LIMITS.pid.d.max}
                className="control-input pid-input"
              />
            </div>
          </div>
          <div className="pid-actions">
            <button 
              className="pid-submit-button" 
              onClick={onPidParamsSubmit}
            >
              应用 PID 参数
            </button>
          </div>
          <div className="speed-limits">
            <span className="limit-label">P范围: 0 ~ 100</span>
            <span className="limit-label">I范围: 0 ~ 100</span>
            <span className="limit-label">D范围: 0 ~ 100</span>
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
  pidParams: PropTypes.shape({
    p: PropTypes.number.isRequired,
    i: PropTypes.number.isRequired,
    d: PropTypes.number.isRequired
  }).isRequired,
  onWheelSpeedChange: PropTypes.func.isRequired,
  onVelocityChange: PropTypes.func.isRequired,
  onPidParamChange: PropTypes.func.isRequired,
  onPidParamsSubmit: PropTypes.func.isRequired
};

MotionControl.displayName = 'MotionControl';