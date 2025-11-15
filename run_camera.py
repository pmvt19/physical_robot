import cv2
import time
import redis
import numpy as np
import depthai as dai

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
    redis_client = redis.Redis(host='localhost', port=6379, db=0)

    pipeline = dai.Pipeline()

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
    stereoOut = stereo.depth.createOutputQueue()

    pipeline.start()
    try:
        print("Running Pipeline")
        while pipeline.isRunning():
            colorFrame : dai.ImgFrame = colorOut.get()
            stereoFrame : dai.ImgFrame = stereoOut.get()

            # rgb_img : np.ndarray = colorFrame.getFrame()
            rgb_img : np.ndarray = colorFrame.getCvFrame() # np.uint8
            stereo_img : np.ndarray = stereoFrame.getFrame() # np.uint16

            # Publish RGB Image Data
            redis_client.set('rgb_img', rgb_img.flatten().tobytes())
            redis_client.set('rgb_img_shape', np.array(rgb_img.shape).tobytes()) # TODO: FIX
            redis_client.set('rgb_img_type', rgb_img.dtype.str)

            # Publish Stereo Image Data
            redis_client.set('stereo_img', stereo_img.tobytes())
            redis_client.set('stereo_img_shape', np.array(stereo_img.shape).tobytes()) # TODO: FIX
            redis_client.set('rgb_img_type', stereo_img.dtype.str)

            # Publish Time Images Were Grabbed
            redis_client.set('camera_time', time.time())
    

    except KeyboardInterrupt:
        print("\nCtrl+C detected. Performing cleanup...")
        print("Cleanup complete. Exiting.")

if __name__ == '__main__':
    start_camera()