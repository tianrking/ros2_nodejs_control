// components/DataMonitor/index.jsx
import { useEffect, useRef, memo } from 'react';
import PropTypes from 'prop-types';
import * as echarts from 'echarts';
import './style.css';

export const DataMonitor = memo(({ sensorData, chartData }) => {
  const chartRef = useRef(null);
  const chartInstanceRef = useRef(null);

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

  // 监听数据变化并更新图表
  useEffect(() => {
    if (chartInstanceRef.current && chartData) {
      chartInstanceRef.current.setOption({
        xAxis: {
          data: chartData.times
        },
        series: [
          {
            name: '左轮实际速度',
            data: chartData.leftWheelFeedback
          },
          {
            name: '右轮实际速度',
            data: chartData.rightWheelFeedback
          },
          {
            name: '左轮期望速度',
            data: chartData.leftWheelTarget
          },
          {
            name: '右轮期望速度',
            data: chartData.rightWheelTarget
          }
        ]
      });
    }
  }, [chartData]);

  return (
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
      <div ref={chartRef} className="chart-container" />
    </div>
  );
});

DataMonitor.propTypes = {
  sensorData: PropTypes.shape({
    wheelLeftFeedback: PropTypes.number.isRequired,
    wheelRightFeedback: PropTypes.number.isRequired
  }).isRequired,
  chartData: PropTypes.shape({
    times: PropTypes.array.isRequired,
    leftWheelFeedback: PropTypes.array.isRequired,
    rightWheelFeedback: PropTypes.array.isRequired,
    leftWheelTarget: PropTypes.array.isRequired,
    rightWheelTarget: PropTypes.array.isRequired
  }).isRequired
};

DataMonitor.displayName = 'DataMonitor';