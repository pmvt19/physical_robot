import React from 'react';
import { LineChart, Line, XAxis, YAxis, Tooltip, ResponsiveContainer, ReferenceLine } from 'recharts';
import { TelemetryFrame } from '../types';

interface IMUChartsProps {
  history: TelemetryFrame[];
}

const ChartCard: React.FC<{ title: string; data: any[]; dataKeys: string[]; colors: string[] }> = ({ title, data, dataKeys, colors }) => (
  <div className="bg-slate-900 rounded-xl border border-slate-800 p-3 shadow-lg flex flex-col h-full min-h-[160px]">
    <h3 className="text-slate-400 text-xs font-bold uppercase tracking-wider mb-2">{title}</h3>
    <div className="flex-1 min-h-0">
      <ResponsiveContainer width="100%" height="100%">
        <LineChart data={data} margin={{ top: 5, right: 5, left: 0, bottom: 0 }}>
          <XAxis dataKey="time" hide />
          <YAxis 
            domain={['auto', 'auto']} 
            tick={{ fill: '#64748b', fontSize: 10 }} 
            tickLine={{ stroke: '#334155' }}
            axisLine={{ stroke: '#334155' }}
            width={35}
          />
          <Tooltip 
            contentStyle={{ backgroundColor: '#1e293b', border: '1px solid #334155', borderRadius: '4px', fontSize: '12px', color: '#f1f5f9' }}
            itemStyle={{ padding: 0 }}
            labelStyle={{ color: '#94a3b8' }}
          />
          <ReferenceLine y={0} stroke="#334155" />
          {dataKeys.map((key, index) => (
            <Line 
              key={key} 
              type="monotone" 
              dataKey={key} 
              stroke={colors[index]} 
              strokeWidth={2} 
              dot={false} 
              isAnimationActive={false} // Disable animation for real-time performance
            />
          ))}
        </LineChart>
      </ResponsiveContainer>
    </div>
    <div className="flex justify-between px-2 mt-1">
        {dataKeys.map((key, i) => (
            <span key={key} className="text-[10px]" style={{ color: colors[i] }}>{key.toUpperCase()}</span>
        ))}
    </div>
  </div>
);

const IMUCharts: React.FC<IMUChartsProps> = ({ history }) => {
  // Format data for Recharts
  const chartData = history.map(frame => ({
    time: frame.state.timestamp,
    accX: frame.imu.accelerometer.x,
    accY: frame.imu.accelerometer.y,
    accZ: frame.imu.accelerometer.z,
    gyroX: frame.imu.gyroscope.x,
    gyroY: frame.imu.gyroscope.y,
    gyroZ: frame.imu.gyroscope.z,
  }));

  return (
    <div className="flex flex-col gap-4 h-full relative">
      {/* Accelerometer */}
      <div className="flex-1 min-h-0">
         <ChartCard 
            title="Accelerometer (m/s²)" 
            data={chartData} 
            dataKeys={['accX', 'accY', 'accZ']} 
            colors={['#ef4444', '#10b981', '#3b82f6']} 
         />
      </div>

      {/* Gyroscope */}
      <div className="flex-1 min-h-0">
        <ChartCard 
            title="Gyroscope (rad/s)" 
            data={chartData} 
            dataKeys={['gyroX', 'gyroY', 'gyroZ']} 
            colors={['#f59e0b', '#8b5cf6', '#ec4899']} 
        />
      </div>
    </div>
  );
};

export default IMUCharts;