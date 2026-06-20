import React from 'react';
import { RobotState } from '../types';

interface StateInfoProps {
  state: RobotState;
}

const MetricBox: React.FC<{ label: string; value: string | number; unit?: string; highlight?: boolean }> = ({ label, value, unit, highlight }) => (
  <div className="bg-slate-950/50 rounded p-2 border border-slate-800 flex flex-col items-center justify-center">
    <span className="text-slate-500 uppercase font-bold tracking-wider mb-0.5 text-[9px]">{label}</span>
    <span className={`font-mono font-medium text-sm ${highlight ? 'text-emerald-400' : 'text-slate-200'}`}>
      {value}
      {unit && <span className="text-slate-600 text-[10px] ml-1">{unit}</span>}
    </span>
  </div>
);

const StateInfo: React.FC<StateInfoProps> = ({ state }) => {
  return (
    <div className="bg-slate-900 rounded-xl border border-slate-800 p-3 shadow-lg flex flex-col h-full">
      <div className="flex justify-between items-center mb-2 shrink-0">
        <h3 className="text-slate-400 text-xs font-bold uppercase tracking-wider">System Status</h3>
        <span className={`px-2 py-0.5 rounded text-[10px] font-bold ${state.mode === 'ERROR' ? 'bg-red-900 text-red-200' : 'bg-emerald-900 text-emerald-200'}`}>
          {state.mode}
        </span>
      </div>

      <div className="grid grid-cols-2 md:grid-cols-4 gap-2 flex-1">
        {/* Positions */}
        <MetricBox label="Left Pos" value={state.leftMotorPosition.toFixed(2)} unit="rad" />
        <MetricBox label="Right Pos" value={state.rightMotorPosition.toFixed(2)} unit="rad" />
        
        {/* Velocities */}
        <MetricBox label="Left Vel" value={state.leftMotorVelocity.toFixed(1)} unit="rad/s" />
        <MetricBox label="Right Vel" value={state.rightMotorVelocity.toFixed(1)} unit="rad/s" />
      </div>
    </div>
  );
};

export default StateInfo;