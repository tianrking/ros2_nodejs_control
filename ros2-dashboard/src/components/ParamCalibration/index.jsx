// components/ParamCalibration/index.jsx
import { memo } from 'react';
import PropTypes from 'prop-types';
import './style.css';

const VEHICLE_TYPES = [
  { value: 'ackermann', label: '阿克曼小车 (2WD1S)' },
  { value: 'differential', label: '两轮差速小车-履带车 (2WD)' },
  { value: 'mecanum', label: '四轮全向车 (4WD)' },
  { value: 'boat', label: '差速船 (DEV)' }
];

const PARAM_CONFIGS = {
  wheelRadius: {
    label: '轮子半径',
    unit: 'm',
    step: 0.001,
    min: 0.01,
    max: 0.5,
    precision: 3
  },
  vehicleWidth: {
    label: '车身宽度',
    unit: 'm',
    step: 0.01,
    min: 0.1,
    max: 2,
    precision: 2
  },
  vehicleLength: {
    label: '车身长度',
    unit: 'm',
    step: 0.01,
    min: 0.1,
    max: 2,
    precision: 2
  }
};

export const ParamCalibration = memo(({
  vehicleType,
  vehicleParams,
  paramsModified,
  submitSuccess,
  onVehicleTypeChange,
  onParamChange,
  onSubmit
}) => {
  const handleParamWheel = (param, e) => {
    e.preventDefault();
    const config = PARAM_CONFIGS[param];
    const delta = e.deltaY > 0 ? -config.step : config.step;
    const newValue = Math.round((vehicleParams[param] + delta) * Math.pow(10, config.precision)) / Math.pow(10, config.precision);
    
    if (newValue >= config.min && newValue <= config.max) {
      onParamChange(param, newValue);
    }
  };

  return (
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
          {Object.entries(PARAM_CONFIGS).map(([param, config]) => (
            <div className="control-input-group" key={param}>
              <label>{config.label} ({config.unit})</label>
              <div className="input-with-unit">
                <input
                  type="number"
                  value={vehicleParams[param]}
                  onChange={(e) => onParamChange(param, parseFloat(e.target.value) || 0)}
                  onWheel={(e) => handleParamWheel(param, e)}
                  step={config.step}
                  min={config.min}
                  max={config.max}
                  className="param-input"
                />
                <span className="unit">{config.unit}</span>
              </div>
              <div className="param-range">
                <span className="range-value">{config.min}</span>
                <span className="range-separator">~</span>
                <span className="range-value">{config.max}</span>
                <span className="unit">{config.unit}</span>
              </div>
            </div>
          ))}
          
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