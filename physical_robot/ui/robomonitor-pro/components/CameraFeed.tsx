import React from 'react';

interface CameraFeedProps {
  rgbUrl: string;
  depthUrl: string;
}

const CameraFeed: React.FC<CameraFeedProps> = ({ rgbUrl, depthUrl }) => {
  return (
    <div className="bg-slate-900 rounded-xl border border-slate-800 p-1 shadow-lg flex flex-col h-full overflow-hidden">
      <div className="px-3 py-2 bg-slate-900 border-b border-slate-800 shrink-0">
        <h3 className="text-slate-400 text-xs font-bold uppercase tracking-wider">Vision System</h3>
      </div>
      
      <div className="flex-1 flex flex-col min-h-0 bg-black">
        {/* RGB Feed */}
        <div className="flex-1 relative overflow-hidden border-b border-slate-800">
           <img src={rgbUrl || undefined} alt="RGB" className="w-full h-full object-contain img-rgb" />
           <div className="absolute top-2 left-2 px-2 py-0.5 bg-black/60 backdrop-blur text-blue-400 text-[10px] font-mono border border-blue-500/30 rounded shadow-sm">
              RGB_CAMERA
           </div>
        </div>

        {/* Depth Feed */}
        <div className="flex-1 relative overflow-hidden">
           <img src={depthUrl || undefined} alt="Depth" className="w-full h-full object-contain img-depth" />
           <div className="absolute top-2 left-2 px-2 py-0.5 bg-black/60 backdrop-blur text-purple-400 text-[10px] font-mono border border-purple-500/30 rounded shadow-sm">
              DEPTH_SENSOR
           </div>
        </div>
      </div>
    </div>
  );
};

export default CameraFeed;