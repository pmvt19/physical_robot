export interface Vector3 {
  x: number;
  y: number;
  z: number;
}

export interface Quaternion {
  x: number;
  y: number;
  z: number;
  w: number;
}

export interface RobotState {
  timestamp: number;
  // Previously used fields (Battery, CPU, RAM) kept in type for backward compatibility
  // but will be ignored in the current UI view.
  batteryVoltage: number; // Volts
  batteryLevel: number; // Percentage
  linearVelocity: number; // m/s
  angularVelocity: number; // rad/s
  cpuUsage: number;
  ramUsage: number;
  wifiSignal: number; // dBm
  mode: 'IDLE' | 'AUTONOMOUS' | 'MANUAL' | 'ERROR';
  
  // New Motor Telemetry
  leftMotorVelocity: number; // rad/s
  rightMotorVelocity: number; // rad/s
  leftMotorPosition: number; // radians
  rightMotorPosition: number; // radians
}

export interface IMUData {
  accelerometer: Vector3;
  gyroscope: Vector3;
  orientation: Quaternion;
}

// Aggregated payload representing the Protobuf message structure
export interface TelemetryFrame {
  id: number;
  state: RobotState;
  imu: IMUData;
  // Lidar is now an array of [x, y] coordinates in meters
  // e.g., [[1.2, 0.5], [1.2, 0.6], ...]
  lidar: number[][]; 
  rgbImage: string; // Base64 or URL
  depthImage: string; // Base64 or URL
}

export enum ConnectionStatus {
  DISCONNECTED = 'DISCONNECTED',
  CONNECTING = 'CONNECTING',
  CONNECTED = 'CONNECTED',
  ERROR = 'ERROR'
}