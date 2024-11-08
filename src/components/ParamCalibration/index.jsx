// src/components/ParamCalibration/index.jsx
import { memo, useState } from 'react';
import PropTypes from 'prop-types';
import './style.css';

const VEHICLE_TYPES = [
  { value: 'ackermann', label: '阿克曼小车 (2WD1S)' },
  { value: 'differential', label: '两轮差速小车-履带车 (2WD)' },
  { value: 'mecanum', label: '四轮全向车 (4WD)' },
  { value: 'boat', label: '差速船 (DEV)' }
];

export const ParamCalibration = memo(({
  vehicleType,
  vehicleParams,
  paramsModified,
  submitSuccess,
  onVehicleTypeChange,
  onParamChange,
  onSubmit
}) => {
  const [isCollapsed, setIsCollapsed] = useState(() => {
    const saved = localStorage.getItem('paramCalibrationCollapsed');
    return saved ? JSON.parse(saved) : false;
  });

  const handleCollapse = () => {
    setIsCollapsed(prev => {
      const newState = !prev;
      localStorage.setItem('paramCalibrationCollapsed', JSON.stringify(newState));
      return newState;
    });
  };

  return (
    <div className="dashboard-card">
      <div className="section-header" onClick={handleCollapse}>
        <h2>参数标定</h2>
        <button className={`collapse-button ${isCollapsed ? 'collapsed' : ''}`}>
          {isCollapsed ? '展开' : '收起'}
        </button>
      </div>

      <div className={`collapsible-content ${isCollapsed ? 'collapsed' : ''}`}>
        <div className="control-grid">
          {/* 载具架构选择 */}
          <div className="control-section">
            <h3>载具架构</h3>
            <div className="control-input-group">
              <div className="vehicle-type-select">
                <select
                  value={vehicleType}
                  onChange={(e) => onVehicleTypeChange(e.target.value)}
                  className="vehicle-select"
                >
                  {VEHICLE_TYPES.map(type => (
                    <option key={type.value} value={type.value}>
                      {type.label}
                    </option>
                  ))}
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
                  onChange={(e) => onParamChange('wheelRadius', parseFloat(e.target.value) || 0)}
                  onWheel={(e) => {
                    e.preventDefault();
                    const delta = e.deltaY > 0 ? -0.001 : 0.001;
                    const newValue = Math.round((vehicleParams.wheelRadius + delta) * 1000) / 1000;
                    if (newValue >= 0.01 && newValue <= 0.5) {
                      onParamChange('wheelRadius', newValue);
                    }
                  }}
                  step="0.001"
                  min="0.01"
                  max="0.5"
                  className="param-input"
                />
                <span className="unit">m</span>
              </div>
              <div className="param-range">
                <span>范围: 0.01 ~ 0.50 m</span>
              </div>
            </div>

            <div className="control-input-group">
              <label>车身宽度 (m)</label>
              <div className="input-with-unit">
                <input
                  type="number"
                  value={vehicleParams.vehicleWidth}
                  onChange={(e) => onParamChange('vehicleWidth', parseFloat(e.target.value) || 0)}
                  onWheel={(e) => {
                    e.preventDefault();
                    const delta = e.deltaY > 0 ? -0.01 : 0.01;
                    const newValue = Math.round((vehicleParams.vehicleWidth + delta) * 100) / 100;
                    if (newValue >= 0.1 && newValue <= 2) {
                      onParamChange('vehicleWidth', newValue);
                    }
                  }}
                  step="0.01"
                  min="0.1"
                  max="2"
                  className="param-input"
                />
                <span className="unit">m</span>
              </div>
              <div className="param-range">
                <span>范围: 0.10 ~ 2.00 m</span>
              </div>
            </div>

            <div className="control-input-group">
              <label>车身长度 (m)</label>
              <div className="input-with-unit">
                <input
                  type="number"
                  value={vehicleParams.vehicleLength}
                  onChange={(e) => onParamChange('vehicleLength', parseFloat(e.target.value) || 0)}
                  onWheel={(e) => {
                    e.preventDefault();
                    const delta = e.deltaY > 0 ? -0.01 : 0.01;
                    const newValue = Math.round((vehicleParams.vehicleLength + delta) * 100) / 100;
                    if (newValue >= 0.1 && newValue <= 2) {
                      onParamChange('vehicleLength', newValue);
                    }
                  }}
                  step="0.01"
                  min="0.1"
                  max="2"
                  className="param-input"
                />
                <span className="unit">m</span>
              </div>
              <div className="param-range">
                <span>范围: 0.10 ~ 2.00 m</span>
              </div>
            </div>

            <div className="params-actions">
              <button
                className={`submit-button ${paramsModified ? 'modified' : ''}`}
                onClick={onSubmit}
                disabled={!paramsModified}
              >
                应用设置
              </button>
              {submitSuccess && (
                <span className="success-message">
                  <i className="success-icon">✓</i>
                  设置已更新
                </span>
              )}
            </div>
          </div>
        </div>
      </div>
    </div>
  );
});

ParamCalibration.propTypes = {
  vehicleType: PropTypes.oneOf(VEHICLE_TYPES.map(t => t.value)).isRequired,
  vehicleParams: PropTypes.shape({
    wheelRadius: PropTypes.number.isRequired,
    vehicleWidth: PropTypes.number.isRequired,
    vehicleLength: PropTypes.number.isRequired
  }).isRequired,
  paramsModified: PropTypes.bool.isRequired,
  submitSuccess: PropTypes.bool.isRequired,
  onVehicleTypeChange: PropTypes.func.isRequired,
  onParamChange: PropTypes.func.isRequired,
  onSubmit: PropTypes.func.isRequired
};

ParamCalibration.displayName = 'ParamCalibration';