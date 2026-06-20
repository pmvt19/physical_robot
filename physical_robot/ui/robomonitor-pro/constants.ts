
export const THEME = {
  colors: {
    primary: '#0ea5e9', // Sky 500
    secondary: '#6366f1', // Indigo 500
    accent: '#f59e0b', // Amber 500
    danger: '#ef4444', // Red 500
    success: '#10b981', // Emerald 500
    background: '#0f172a', // Slate 900
    surface: '#1e293b', // Slate 800
  }
};

export const LIDAR_CONFIG = {
  maxRange: 7, // meters
  angleStart: -Math.PI,
  angleEnd: Math.PI,
  numPoints: 360,
};

export const MAX_CHART_HISTORY = 60; // Keep last 60 frames for charts

export const DEFAULT_GEMINI_MODEL = 'gemini-3-flash-preview';

// Point this to your Python Server IP
export const ROBOT_WS_URL = 'ws://localhost:9090';
