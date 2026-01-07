import asyncio
import websockets
import json
import time
import math
import base64

# Import libraries for array handling
try:
    import numpy as np
    import cv2
except ImportError:
    print("Please install numpy and opencv: pip install numpy opencv-python")
    exit(1)

from robot import Robot

def encode_image_to_base64(numpy_img):
    """
    Compresses a numpy image (H, W, C) into JPEG and returns a Base64 string.
    """
    # 1. Encode into JPEG buffer
    success, buffer = cv2.imencode('.jpg', numpy_img, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
    if not success:
        return ""
    
    # 2. Convert to base64 string
    b64_str = base64.b64encode(buffer).decode('utf-8')
    return f"data:image/jpeg;base64,{b64_str}"

async def handler(websocket, path=None):
    print(f"Client connected from {websocket.remote_address}")

    robot = Robot(connection='client')

    try:
        while True:
            t = time.time()
            
            # --- 1. LIDAR DATA (Numpy -> List of [x, y]) ---
            coords, _ = robot.read_lidar_updated(wait_for_updated_reading=False)
            coords = coords / 1000.0  # Convert mm to meters
            coords = coords[:, :2]

            # Rotate Coords 90 degrees clockwise
            theta = math.pi / 2  # 90 degrees in radians
            rotation_matrix = np.array([[np.cos(theta), -np.sin(theta)],
                                         [np.sin(theta), np.cos(theta)]])
            coords = coords.dot(rotation_matrix.T)

            lidar_payload = coords[:, :2].tolist()

            # --- 2. IMAGE DATA (Numpy -> Base64) ---
            rgb_numpy, depth_numpy = robot.read_rgb_camera()
            # rgb_numpy = robot.read_depth_camera()
            # depth_numpy = robot.read_depth_camera()

            # Colorize depth for better visualization
            depth_numpy = cv2.applyColorMap(cv2.convertScaleAbs(depth_numpy, alpha=0.03), cv2.COLORMAP_JET)

            # Encode
            rgb_image_payload = encode_image_to_base64(rgb_numpy)
            depth_image_payload = encode_image_to_base64(depth_numpy)

            accel_data, gyro_data = robot.read_imu()

            # --- 3. CONSTRUCT PAYLOAD ---
            data = {
                "id": int(t * 10),
                "state": {
                    "timestamp": int(t * 1000),
                    "batteryVoltage": 24.5,
                    "batteryLevel": 80.0,
                    "linearVelocity": 0.0,
                    "angularVelocity": 0.0,
                    "cpuUsage": 40.0,
                    "ramUsage": 50.0,
                    "wifiSignal": -45,
                    "mode": "AUTONOMOUS",
                    "leftMotorVelocity": 0.0,
                    "rightMotorVelocity": 0.0,
                    "leftMotorPosition": 0.0,
                    "rightMotorPosition": 0.0
                },
                "imu": {
                    "accelerometer": {"x": accel_data[0], "y": accel_data[1], "z": accel_data[2]},
                    "gyroscope": {"x": gyro_data[0], "y": gyro_data[1], "z": gyro_data[2]},
                    "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}
                },
                "lidar": lidar_payload,    # Sending [[x,y], [x,y]...]
                "rgbImage": rgb_image_payload, # Sending "data:image/jpeg;base64,..."
                "depthImage": depth_image_payload # Reusing for demo
            }

            await websocket.send(json.dumps(data))
            await asyncio.sleep(0.1) # 10 FPS
            
    except websockets.exceptions.ConnectionClosed:
        print("Client disconnected")
    # except Exception as e:
    #     print(f"Error: {e}")

async def main():
    async with websockets.serve(handler, "0.0.0.0", 9090):
        print("WebSocket Server running on ws://0.0.0.0:9090")
        await asyncio.Future()

if __name__ == "__main__":
    asyncio.run(main())