import { GoogleGenAI } from "@google/genai";
import { TelemetryFrame, RobotState, IMUData } from "../types";
import { DEFAULT_GEMINI_MODEL } from "../constants";

export const analyzeTelemetry = async (frame: TelemetryFrame): Promise<string> => {
  if (!process.env.API_KEY) {
    return "Error: API Key is missing. Please check your environment configuration.";
  }

  const ai = new GoogleGenAI({ apiKey: process.env.API_KEY });

  // Prepare a condensed summary for the prompt to save tokens/complexity
  const summary = {
    battery: `${frame.state.batteryLevel.toFixed(1)}% (${frame.state.batteryVoltage.toFixed(2)}V)`,
    velocity: `Linear: ${frame.state.linearVelocity.toFixed(2)} m/s, Angular: ${frame.state.angularVelocity.toFixed(2)} rad/s`,
    cpu: `${frame.state.cpuUsage.toFixed(1)}%`,
    mode: frame.state.mode,
    imu_accel: frame.imu.accelerometer,
    warning: frame.state.batteryLevel < 20 ? "Battery Low" : "None"
  };

  const prompt = `
    You are an advanced robot diagnostics AI. Analyze the following telemetry snapshot from a mobile robot. 
    
    Telemetry Data:
    ${JSON.stringify(summary, null, 2)}

    Identify any potential issues (like high CPU, low battery, irregular IMU spikes compared to stationary gravity) or confirm all systems are nominal. 
    Provide a concise status report in Markdown format. Limit response to 3 sentences.
  `;

  try {
    const response = await ai.models.generateContent({
      model: DEFAULT_GEMINI_MODEL,
      contents: prompt,
    });
    
    return response.text || "No analysis generated.";
  } catch (error) {
    console.error("Gemini API Error:", error);
    return "Diagnostic system currently unavailable. Check network or API quota.";
  }
};