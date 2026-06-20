import React, { useState } from 'react';
import { robotService } from '../services/robotService';
import { ConnectionStatus } from '../types';
import { X, Cog, RefreshCw, AlertTriangle, HelpCircle } from 'lucide-react';

interface ConnectionSettingsModalProps {
  isOpen: boolean;
  onClose: () => void;
  status: ConnectionStatus;
}

export const ConnectionSettingsModal: React.FC<ConnectionSettingsModalProps> = ({
  isOpen,
  onClose,
  status
}) => {
  const [url, setUrl] = useState(robotService.getUrl());
  const [autoWss, setAutoWss] = useState(robotService.getAutoWssUpgrade());

  if (!isOpen) return null;

  const presets = [
    { label: 'Local IPv4 (Recommended)', value: 'ws://127.0.0.1:9090' },
    { label: 'Localhost (Legacy)', value: 'ws://localhost:9090' },
    { label: 'Secure Local WSS (SSL)', value: 'wss://127.0.0.1:9090' },
  ];

  const handleApply = (e?: React.FormEvent) => {
    if (e) e.preventDefault();
    robotService.setUrl(url);
    robotService.setAutoWssUpgrade(autoWss);
    onClose();
  };

  const handlePresetSelect = (presetValue: string) => {
    setUrl(presetValue);
  };

  return (
    <div id="connection-settings-overlay" className="fixed inset-0 z-50 flex items-center justify-center bg-slate-950/80 backdrop-blur-sm p-4 overflow-y-auto">
      <div 
        id="connection-settings-container" 
        className="w-full max-w-2xl bg-slate-900 border border-slate-800 rounded-xl shadow-2xl overflow-hidden flex flex-col my-8"
      >
        {/* Header */}
        <div className="flex items-center justify-between px-6 py-4 border-b border-slate-800 bg-slate-900/50">
          <div className="flex items-center gap-2.5">
            <Cog className="w-5 h-5 text-blue-400 animate-spin-slow" />
            <h2 className="text-base font-bold text-slate-100 uppercase tracking-wider">Uplink Configuration</h2>
          </div>
          <button 
            onClick={onClose}
            className="p-1.5 rounded-lg text-slate-400 hover:text-slate-100 hover:bg-slate-800 transition"
          >
            <X className="w-5 h-5" />
          </button>
        </div>

        {/* Form Content */}
        <form onSubmit={handleApply} className="p-6 space-y-6 flex-1 overflow-y-auto">
          
          {/* Status Bar */}
          <div className="flex items-center justify-between p-4 bg-slate-950 rounded-lg border border-slate-800/80">
            <div className="space-y-1">
              <span className="text-xs text-slate-500 font-medium">Uplink Status</span>
              <div className="flex items-center gap-2">
                <span className={`w-2 h-2 rounded-full ${
                  status === ConnectionStatus.CONNECTED ? 'bg-emerald-500 animate-pulse' : 
                  status === ConnectionStatus.CONNECTING ? 'bg-amber-500' : 'bg-red-500'
                }`} />
                <span className="text-sm font-mono font-bold text-slate-200">{status}</span>
              </div>
            </div>
            
            <button
              type="button"
              onClick={() => robotService.connect()}
              className="flex items-center gap-1.5 text-xs font-semibold px-3 py-1.5 rounded bg-blue-500 hover:bg-blue-600 active:bg-blue-700 text-slate-100 transition shadow-lg shadow-blue-500/10"
            >
              <RefreshCw className="w-3.5 h-3.5" /> Force Retry
            </button>
          </div>

          {/* Connection Error Diagnostic Banner */}
          {status !== ConnectionStatus.CONNECTED && (
            <div className="p-4 bg-amber-500/10 border border-amber-500/20 rounded-lg text-amber-200/90 text-xs leading-relaxed space-y-2">
              <div className="flex items-center gap-2 text-amber-400 font-bold">
                <AlertTriangle className="w-4 h-4 shrink-0" />
                <span>Websocket Connection Trouble?</span>
              </div>
              <p>
                Because this telemetry console serves files over secure <b className="text-amber-300">HTTPS</b>, web browsers block unencrypted mixed-content (<span className="font-mono text-amber-300">ws://</span>) calls to remote hosts.
              </p>
              <p>
                If your physical server lacks SSL, standard browsers will either convert your connection to <span className="font-mono text-amber-300">wss://</span> automatically or drop the socket during handshake, generating a <code className="text-amber-300 font-mono bg-amber-500/10 px-1 rounded">websockets.exceptions.InvalidMessage</code> error due to reading SSL binary packets without support.
              </p>
            </div>
          )}

          {/* URL Entry */}
          <div className="space-y-2">
            <label className="block text-xs font-semibold text-slate-400 uppercase tracking-wider">Robot Websocket Endpoint URL</label>
            <input 
              type="text" 
              value={url}
              onChange={(e) => setUrl(e.target.value)}
              className="w-full bg-slate-950 border border-slate-800 focus:border-blue-500 focus:ring-1 focus:ring-blue-500 rounded-lg px-3.5 py-2.5 text-sm font-mono text-slate-200 placeholder-slate-700 outline-none transition"
              placeholder="ws://127.0.0.1:9090"
              required
            />
          </div>

          {/* Presets */}
          <div className="space-y-2">
            <span className="block text-xs font-semibold text-slate-400 uppercase tracking-wider">Quick Presets</span>
            <div className="grid grid-cols-1 sm:grid-cols-3 gap-2">
              {presets.map((p, i) => (
                <button
                  key={i}
                  type="button"
                  onClick={() => handlePresetSelect(p.value)}
                  className={`px-3 py-2 text-left rounded-lg border text-xs transition flex flex-col justify-between ${
                    url === p.value 
                      ? 'bg-blue-500/10 border-blue-500/50 text-blue-300' 
                      : 'bg-slate-950 border-slate-800/80 text-slate-400 hover:border-slate-700 hover:text-slate-300'
                  }`}
                >
                  <span className="font-medium mb-1">{p.label}</span>
                  <span className="font-mono text-[10px] opacity-75">{p.value}</span>
                </button>
              ))}
            </div>
          </div>

          {/* Secure Handshake Settings */}
          <div className="space-y-4 pt-2">
            <div className="flex items-start justify-between gap-4 p-4 bg-slate-950 rounded-lg border border-slate-800/80">
              <div className="space-y-1">
                <div className="flex items-center gap-2">
                  <span className="text-xs font-semibold text-slate-300 uppercase tracking-wider">Secure SSL/TLS Auto-Upgrade (WSS)</span>
                  <HelpCircle className="w-3.5 h-3.5 text-slate-500" title="When active under HTTPS, ws:// is automatically resolved/upgraded to wss:// for public hosts." />
                </div>
                <p className="text-xs text-slate-500 leading-relaxed">
                  Automatically upgrades unencrypted <span className="font-mono text-blue-400">ws://</span> strings to <span className="font-mono text-emerald-400">wss://</span> when under secure pages (except loopbacks <span className="font-mono text-slate-400">localhost/127.0.0.1</span>).
                </p>
              </div>
              <label className="relative inline-flex items-center cursor-pointer select-none">
                <input 
                  type="checkbox" 
                  checked={autoWss} 
                  onChange={(e) => setAutoWss(e.target.checked)}
                  className="sr-only peer" 
                />
                <div className="w-11 h-6 bg-slate-800 rounded-full peer peer-checked:after:translate-x-full after:content-[''] after:absolute after:top-0.5 after:left-[2px] after:bg-slate-300 after:rounded-full after:h-5 after:w-5 after:transition-all peer-checked:bg-blue-500 border border-slate-700"></div>
              </label>
            </div>
          </div>

        </form>

        {/* Footer */}
        <div className="px-6 py-4 border-t border-slate-800 bg-slate-900/50 flex justify-end gap-3 shrink-0">
          <button
            type="button"
            onClick={onClose}
            className="px-4 py-2 rounded-lg text-xs font-semibold text-slate-400 hover:text-slate-200 hover:bg-slate-800 transition"
          >
            Cancel
          </button>
          <button
            type="button"
            onClick={() => handleApply()}
            className="px-4 py-2 rounded-lg text-xs font-semibold bg-blue-500 hover:bg-blue-600 active:bg-blue-700 text-slate-100 transition shadow-lg shadow-blue-500/15"
          >
            Save & Reconnect
          </button>
        </div>
      </div>
    </div>
  );
};
