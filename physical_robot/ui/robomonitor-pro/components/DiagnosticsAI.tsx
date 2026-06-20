import React, { useState } from 'react';
import { TelemetryFrame } from '../types';
import { analyzeTelemetry } from '../services/geminiService';

interface DiagnosticsAIProps {
  currentFrame: TelemetryFrame | null;
}

const DiagnosticsAI: React.FC<DiagnosticsAIProps> = ({ currentFrame }) => {
  const [analysis, setAnalysis] = useState<string | null>(null);
  const [loading, setLoading] = useState(false);

  const handleAnalyze = async () => {
    if (!currentFrame) return;
    
    setLoading(true);
    setAnalysis(null);
    
    try {
      const result = await analyzeTelemetry(currentFrame);
      setAnalysis(result);
    } catch (e) {
      setAnalysis("Failed to connect to AI Diagnostics service.");
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="bg-gradient-to-br from-indigo-900/40 to-slate-900 rounded-xl border border-indigo-500/30 p-4 shadow-lg flex flex-col">
      <div className="flex justify-between items-center mb-3">
        <h3 className="text-indigo-400 text-xs font-bold uppercase tracking-wider flex items-center gap-2">
            <svg className="w-4 h-4" fill="currentColor" viewBox="0 0 24 24"><path d="M12 2C6.48 2 2 6.48 2 12s4.48 10 10 10 10-4.48 10-10S17.52 2 12 2zm0 18c-4.41 0-8-3.59-8-8s3.59-8 8-8 8 3.59 8 8-3.59 8-8 8zm-1-13h2v6h-2zm0 8h2v2h-2z"/></svg>
            Gemini Diagnostics
        </h3>
        <button 
            onClick={handleAnalyze}
            disabled={loading || !currentFrame}
            className={`px-3 py-1.5 rounded-md text-xs font-bold transition-colors ${
                loading 
                ? 'bg-slate-700 text-slate-400 cursor-not-allowed' 
                : 'bg-indigo-600 hover:bg-indigo-500 text-white shadow-lg shadow-indigo-500/20'
            }`}
        >
            {loading ? 'Analyzing...' : 'Run Diagnostics'}
        </button>
      </div>

      <div className="flex-1 bg-slate-950/50 rounded-lg p-3 border border-indigo-500/10 min-h-[80px] text-sm text-slate-300 font-mono leading-relaxed overflow-y-auto max-h-32">
        {analysis ? (
            <div className="prose prose-invert prose-sm">
                {analysis.split('\n').map((line, i) => <p key={i} className="mb-1">{line}</p>)}
            </div>
        ) : (
            <div className="flex items-center justify-center h-full text-slate-600 italic text-xs">
                Waiting for manual trigger...
            </div>
        )}
      </div>
    </div>
  );
};

export default DiagnosticsAI;