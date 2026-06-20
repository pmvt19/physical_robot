import React, { useEffect, useState } from 'react';
import { robotService } from './services/robotService';
import { TelemetryFrame, ConnectionStatus } from './types';
import { MAX_CHART_HISTORY } from './constants';
import { Settings } from 'lucide-react';

// Components
import LidarVisualizer from './components/LidarVisualizer';
import StateInfo from './components/StateInfo';
import CameraFeed from './components/CameraFeed';
import IMUCharts from './components/IMUCharts';
import { ConnectionSettingsModal } from './components/ConnectionSettingsModal';

const App: React.FC = () => {
  const [status, setStatus] = useState<ConnectionStatus>(ConnectionStatus.DISCONNECTED);
  const [frame, setFrame] = useState<TelemetryFrame | null>(null);
  const [history, setHistory] = useState<TelemetryFrame[]>([]);
  const [showSettings, setShowSettings] = useState(false);
  
  useEffect(() => {
    // 1. Subscribe to connection status
    const statusUnsub = robotService.onStatusChange(setStatus);
    
    // 2. Subscribe to telemetry data
    const dataUnsub = robotService.subscribe((newFrame) => {
      setFrame(newFrame);
      setHistory(prev => {
        const newHistory = [...prev, newFrame];
        if (newHistory.length > MAX_CHART_HISTORY) {
          return newHistory.slice(newHistory.length - MAX_CHART_HISTORY);
        }
        return newHistory;
      });
    });

    // 3. Initiate connection
    robotService.connect();

    // Cleanup
    return () => {
      statusUnsub();
      dataUnsub();
      robotService.disconnect();
    };
  }, []);

  return (
    <div className="flex flex-col h-screen bg-slate-950 text-slate-200 font-sans">
      {/* Header */}
      <header className="h-14 bg-slate-900 border-b border-slate-800 flex items-center justify-between px-6 shadow-md z-10 shrink-0">
        <div className="flex items-center gap-3">
            <div className="w-8 h-8 rounded bg-gradient-to-tr from-blue-500 to-cyan-400 flex items-center justify-center text-slate-900 font-bold text-lg">R</div>
            <h1 className="font-bold text-lg tracking-tight text-slate-100">RoboMonitor <span className="text-blue-400 font-light">Pro</span></h1>
        </div>
        
        <div className="flex items-center gap-4">
            <button 
              onClick={() => setShowSettings(true)}
              className="flex items-center gap-2 bg-slate-800 hover:bg-slate-700 hover:text-slate-100 transition px-3.5 py-1.5 rounded-full border border-slate-700 cursor-pointer text-slate-300 font-sans shadow-inner outline-none"
              title="Configure Uplink Connection"
            >
                <div className={`w-2 h-2 rounded-full ${
                    status === ConnectionStatus.CONNECTED ? 'bg-emerald-500 animate-pulse' : 
                    status === ConnectionStatus.CONNECTING ? 'bg-amber-500' : 'bg-red-500'
                }`} />
                <span className="text-xs font-mono font-bold tracking-wide">{status}</span>
                <Settings className="w-3.5 h-3.5 ml-0.5 text-slate-400" />
            </button>
        </div>
      </header>

      {/* Main Content Grid */}
      <main className="flex-1 overflow-y-auto lg:overflow-hidden p-4">
        {frame ? (
            <div className="grid grid-cols-12 gap-4 lg:h-full">
                
                {/* Column 1: Camera Feed (Left) */}
                <div className="col-span-12 lg:col-span-3 h-[300px] lg:h-full">
                    <CameraFeed 
                      rgbUrl={frame.rgbImage} 
                      depthUrl={frame.depthImage} 
                    />
                </div>

                {/* Column 2: Lidar (Center - Large) + State Info (Bottom - Small) */}
                <div className="col-span-12 lg:col-span-6 flex flex-col gap-4 h-[500px] lg:h-full">
                    <div className="flex-1 min-h-0">
                         <LidarVisualizer data={frame.lidar} />
                    </div>
                    <div className="h-auto shrink-0">
                         <StateInfo state={frame.state} />
                    </div>
                </div>

                {/* Column 3: Charts (Right) */}
                <div className="col-span-12 lg:col-span-3 h-[400px] lg:h-full">
                    <IMUCharts history={history} />
                </div>

            </div>
        ) : (
            <div className="h-full flex flex-col items-center justify-center text-slate-500 gap-4">
                <div className="animate-spin rounded-full h-12 w-12 border-b-2 border-blue-500"></div>
                <p className="font-medium text-slate-305">Establishing secure uplink to robot client...</p>
                <p className="text-xs text-slate-600">Attempting to align active state protocols</p>
                
                <button
                  onClick={() => setShowSettings(true)}
                  className="mt-3 text-xs font-semibold px-4 py-2 bg-slate-800 hover:bg-slate-700 hover:text-slate-100 text-slate-200 border border-slate-700/80 rounded-lg transition flex items-center gap-2 cursor-pointer shadow-lg outline-none"
                >
                  <Settings className="w-4 h-4 text-blue-400 animate-spin-slow" /> Connection Settings & diagnostics
                </button>
            </div>
        )}
      </main>

      {/* Connection Settings Modal Overlay */}
      <ConnectionSettingsModal 
        isOpen={showSettings} 
        onClose={() => setShowSettings(false)} 
        status={status} 
      />
    </div>
  );
};

export default App;