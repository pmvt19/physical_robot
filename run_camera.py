import cv2
import time
import redis
import numpy as np
import depthai as dai

# Connect to Redis
redis_client = redis.Redis(host='localhost', port=6379, db=0)

def processDepthFrame(depthFrame):
    depth_downscaled = depthFrame[::4]
    if np.all(depth_downscaled == 0):
        min_depth = 0
    else:
        min_depth = np.percentile(depth_downscaled[depth_downscaled != 0], 1)
    max_depth = np.percentile(depth_downscaled, 99)
    depthFrameColor = np.interp(depthFrame, (min_depth, max_depth), (0, 255)).astype(np.uint8)
    return cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_HOT)

def start_camera():
    with dai.Pipeline() as pipeline:
        color = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        monoLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
        monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
        stereo = pipeline.create(dai.node.StereoDepth)

        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
        stereo.setOutputSize(640, 480)

        colorCamOut = color.requestOutput((640, 480))
        monoLeftOut = monoLeft.requestOutput((640, 480))
        monoRightOut = monoRight.requestOutput((640, 480))

        monoLeftOut.link(stereo.left)
        monoRightOut.link(stereo.right)

        colorOut = colorCamOut.createOutputQueue()
        leftOut = monoLeftOut.createOutputQueue()
        rightOut = monoRightOut.createOutputQueue()
        stereoOut = stereo.depth.createOutputQueue()

        pipeline.start()
        try:
            while pipeline.isRunning():
                colorFrame : dai.ImgFrame = colorOut.get()
                stereoFrame : dai.ImgFrame = stereoOut.get()

                rgb_img : np.ndarray = colorFrame.getFrame()
                stereo_img : np.ndarray = stereoFrame.getFrame()

                redis_client.set('rgb_img', rgb_img.tobytes())
                redis_client.set('stereo_img', stereo_img.tobytes())
                redis_client.set('time', time.time())

        except KeyboardInterrupt:
            print("\nCtrl+C detected. Performing cleanup...")
            print("Cleanup complete. Exiting.")
        
        pipeline.stop()

if __name__ == '__main__':
    start_camera()