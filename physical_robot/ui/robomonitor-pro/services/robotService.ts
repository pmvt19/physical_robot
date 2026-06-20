import { TelemetryFrame, ConnectionStatus } from '../types';
import { ROBOT_WS_URL } from '../constants';

type TelemetryCallback = (frame: TelemetryFrame) => void;
type StatusCallback = (status: ConnectionStatus) => void;

// 1. Define Default State
const DEFAULT_FRAME: TelemetryFrame = {
  id: 0,
  state: {
    timestamp: 0,
    batteryVoltage: 0,
    batteryLevel: 0,
    linearVelocity: 0,
    angularVelocity: 0,
    cpuUsage: 0,
    ramUsage: 0,
    wifiSignal: 0,
    mode: 'IDLE',
    leftMotorVelocity: 0,
    rightMotorVelocity: 0,
    leftMotorPosition: 0,
    rightMotorPosition: 0
  },
  imu: {
    accelerometer: { x: 0, y: 0, z: 0 },
    gyroscope: { x: 0, y: 0, z: 0 },
    orientation: { x: 0, y: 0, z: 0, w: 1 }
  },
  lidar: [],
  rgbImage: '', 
  depthImage: ''
};

// 2. Deep Merge Utility
function deepMerge(target: any, source: any): any {
  if (typeof source !== 'object' || source === null) {
    return source;
  }
  if (Array.isArray(source)) {
    return source;
  }
  if (typeof target !== 'object' || target === null || Array.isArray(target)) {
    return source;
  }

  const output = { ...target };
  Object.keys(source).forEach(key => {
    if (key in target) {
      output[key] = deepMerge(target[key], source[key]);
    } else {
      output[key] = source[key];
    }
  });
  return output;
}

class RobotService {
  private status: ConnectionStatus = ConnectionStatus.DISCONNECTED;
  private socket: WebSocket | null = null;
  private listeners: TelemetryCallback[] = [];
  private statusListeners: StatusCallback[] = [];
  private currentFrame: TelemetryFrame = DEFAULT_FRAME;
  private reconnectInterval: number | null = null;
  private shouldReconnect = false;
  
  // Dynamic WebSocket configuration with localStorage persistence
  private wsUrl: string = localStorage.getItem('robo_monitor_ws_url') || ROBOT_WS_URL;
  private autoWssUpgrade: boolean = localStorage.getItem('robo_monitor_auto_wss_upgrade') !== 'false';

  public getUrl(): string {
    return this.wsUrl;
  }

  public setUrl(newUrl: string) {
    if (newUrl === this.wsUrl) return;
    this.wsUrl = newUrl;
    localStorage.setItem('robo_monitor_ws_url', newUrl);
    console.log(`Uplink: Updated target URL to ${newUrl}`);
    if (this.shouldReconnect) {
      this.reconnect();
    }
  }

  public getAutoWssUpgrade(): boolean {
    return this.autoWssUpgrade;
  }

  public setAutoWssUpgrade(value: boolean) {
    if (value === this.autoWssUpgrade) return;
    this.autoWssUpgrade = value;
    localStorage.setItem('robo_monitor_auto_wss_upgrade', String(value));
    console.log(`Uplink: Set autoWssUpgrade to ${value}`);
    if (this.shouldReconnect) {
      this.reconnect();
    }
  }

  private reconnect() {
    this.disconnect();
    // Re-enable and reconnect
    this.shouldReconnect = true;
    this.connect();
  }

  /**
   * Resolves the WebSocket URL based on current security context.
   * If the page is HTTPS, ws:// will fail, so we attempt an upgrade to wss://.
   */
  private getResolvedUrl(url: string): string {
    const isSecure = window.location.protocol === 'https:';
    if (!isSecure || !url.startsWith('ws:')) {
      return url;
    }

    // Allow user to completely bypass HTTPS upgrades if they want to force plaintext ws://
    if (!this.autoWssUpgrade) {
      return url;
    }

    try {
      // Parse the URL to extract the hostname
      const urlObj = new URL(url);
      const hostname = urlObj.hostname.toLowerCase();

      const isLoopback = 
        hostname === 'localhost' || 
        hostname === '127.0.0.1' || 
        hostname === '[::1]' || 
        hostname === '::1';

      const isLocalIp = 
        hostname.startsWith('192.168.') || 
        hostname.startsWith('10.') || 
        /^172\.(1[6-9]|2[0-9]|3[0-1])\./.test(hostname) ||
        hostname.startsWith('169.254.'); // Link-local space

      const isLocalDomain = hostname.endsWith('.local');

      if (isLoopback || isLocalIp || isLocalDomain) {
        console.log(`Uplink: Preserving plaintext WebSocket for local/private host "${hostname}" under HTTPS.`);
        return url;
      }
    } catch (e) {
      // Fallback string-matching if URL parsing fails
      if (url.includes('localhost') || url.includes('127.0.0.1') || url.includes('[::1]') || url.includes('.local')) {
        return url;
      }
    }

    console.warn('Mixed Content Warning: Upgrading WebSocket to secure "wss" because page is HTTPS.');
    return url.replace('ws:', 'wss:');
  }

  public connect() {
    if (this.status === ConnectionStatus.CONNECTED || this.status === ConnectionStatus.CONNECTING) {
      return;
    }

    this.shouldReconnect = true;
    this.updateStatus(ConnectionStatus.CONNECTING);
    
    const targetUrl = this.getResolvedUrl(this.wsUrl);
    
    try {
      console.log(`Uplink: Attempting connection to ${targetUrl}`);
      this.socket = new WebSocket(targetUrl);
      
      this.socket.onopen = () => {
        console.log("%c✓ Uplink Established", "color: #10b981; font-weight: bold;");
        this.updateStatus(ConnectionStatus.CONNECTED);
        if (this.reconnectInterval) {
          window.clearInterval(this.reconnectInterval);
          this.reconnectInterval = null;
        }
      };

      this.socket.onmessage = (event) => {
        try {
          let incomingData: any;
          if (typeof event.data === "string") {
            incomingData = JSON.parse(event.data);
          } else {
             return;
          }
          this.currentFrame = deepMerge(this.currentFrame, incomingData);
          this.listeners.forEach(cb => cb(this.currentFrame));
        } catch (e) {
          console.error("Telemetry parse error:", e);
        }
      };

      this.socket.onclose = (event) => {
        console.warn(`WebSocket closed. Code: ${event.code}, Reason: ${event.reason || 'Not provided'}`);
        this.updateStatus(ConnectionStatus.DISCONNECTED);
        this.socket = null;
        this.attemptReconnect();
      };

      this.socket.onerror = (err) => {
        // Log as a structured object to avoid [object Object] stringification in console
        console.error("WebSocket Connection Failed", {
          url: targetUrl,
          readyState: this.socket?.readyState,
          protocol: window.location.protocol,
          help: "Check if the robot server is running and reachable at the target IP."
        });
        
        this.updateStatus(ConnectionStatus.ERROR);
      };

    } catch (e) {
      console.error("WebSocket initialization failed:", e);
      this.updateStatus(ConnectionStatus.ERROR);
      this.attemptReconnect();
    }
  }

  public disconnect() {
    this.shouldReconnect = false;
    if (this.reconnectInterval) {
      window.clearInterval(this.reconnectInterval);
      this.reconnectInterval = null;
    }
    if (this.socket) {
      this.socket.close();
      this.socket = null;
    }
    this.updateStatus(ConnectionStatus.DISCONNECTED);
  }

  public subscribe(callback: TelemetryCallback) {
    this.listeners.push(callback);
    callback(this.currentFrame);
    return () => {
      this.listeners = this.listeners.filter(cb => cb !== callback);
    };
  }

  public onStatusChange(callback: StatusCallback) {
    this.statusListeners.push(callback);
    return () => {
      this.statusListeners = this.statusListeners.filter(cb => cb !== callback);
    };
  }

  private updateStatus(newStatus: ConnectionStatus) {
    this.status = newStatus;
    this.statusListeners.forEach(cb => cb(newStatus));
  }

  private attemptReconnect() {
    if (!this.shouldReconnect || this.reconnectInterval) return;

    this.reconnectInterval = window.setInterval(() => {
      if (this.status === ConnectionStatus.DISCONNECTED || this.status === ConnectionStatus.ERROR) {
        console.log("Uplink: Attempting automatic reconnection...");
        this.connect();
      }
    }, 3000);
  }
}

export const robotService = new RobotService();