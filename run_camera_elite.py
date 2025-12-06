import depthai as dai
import rerun as rr
import numpy as np
import time
import cv2
import redis

redis_client = redis.Redis(host='localhost', port=6379, db=0)
with dai.Pipeline() as p:
    # fps = 30
    # Define sources and outputs
    left = p.create(dai.node.Camera)
    right = p.create(dai.node.Camera)
    color = p.create(dai.node.Camera)
    stereo = p.create(dai.node.StereoDepth)
    rgbd = p.create(dai.node.RGBD).build()

    imu = p.create(dai.node.IMU)

    # enable ACCELEROMETER_RAW at 500 hz rate
    imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, 480)

    # enable GYROSCOPE_RAW at 400 hz rate
    imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, 400)

    imu.setBatchReportThreshold(1)
    imu.setMaxBatchReports(10)

    print("Done defining nodes")
    align = None
    color.build()

    left.build(dai.CameraBoardSocket.CAM_B)
    right.build(dai.CameraBoardSocket.CAM_C)
    out = None

    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
    stereo.setRectifyEdgeFillColor(0)
    stereo.enableDistortionCorrection(True)
    print("Done building cameras and stereo")

    resolution = (640, 400)

    # Linking
    left.requestOutput(resolution).link(stereo.left)
    right.requestOutput(resolution).link(stereo.right)
    print("done linking mono cameras to stereo")
    platform = p.getDefaultDevice().getPlatform()
    out = color.requestOutput(
        resolution, dai.ImgFrame.Type.RGB888i, dai.ImgResizeMode.CROP, 30, True
    )
    stereo.depth.link(rgbd.inDepth)
    out.link(stereo.inputAlignTo)
    out.link(rgbd.inColor)

    pclOut = rgbd.pcl.createOutputQueue()
    rgbdOut = rgbd.rgbd.createOutputQueue()
    imuOut = imu.out.createOutputQueue(maxSize=1, blocking=False)

    print("About To Start Pipeline")
    p.start()
    print("Pipeline Started")
    while p.isRunning():
        coords, colors = pclOut.get().getPointsRGB()
        rgb_img = rgbdOut.get().getRGBFrame().getCvFrame()
        depth_img = rgbdOut.get().getDepthFrame().getCvFrame()

        coords = np.stack((coords[:, 0], coords[:, 1], coords[:, 2]), axis=1)

        # Publish RGB Image Data
        redis_client.set('rgb_img', rgb_img.flatten().tobytes())
        redis_client.set('rgb_img_shape', np.array(rgb_img.shape).tobytes()) # TODO: FIX
        redis_client.set('rgb_img_type', rgb_img.dtype.str)

        # Publish Stereo Image Data
        redis_client.set('stereo_img', depth_img.tobytes())
        redis_client.set('stereo_img_shape', np.array(depth_img.shape).tobytes()) # TODO: FIX
        redis_client.set('rgb_img_type', depth_img.dtype.str)

        # Publish Time Images Were Grabbed
        redis_client.set('camera_time', time.time())

        # Publish Point Cloud Data - Coords
        redis_client.set('pcl_coords', coords.tobytes())
        redis_client.set('pcl_coords_shape', np.array(coords.shape).tobytes())
        redis_client.set('pcl_coords_type', coords.dtype.str)

        # Publish Point Cloud Data - Colors
        redis_client.set('pcl_colors', colors.tobytes())
        redis_client.set('pcl_colors_shape', np.array(colors.shape).tobytes())
        redis_client.set('pcl_colors_type', colors.dtype.str)

        # Grab IMU Data
        imuData = imuOut.get()

        # Get Single IMU Packet
        imuPacket = imuData.packets[0]

        # Separate Accelerometer and Gyroscope Values
        acceleroValues = imuPacket.acceleroMeter
        gyroValues = imuPacket.gyroscope
        
        # Get Timestamps
        acceleroTs = acceleroValues.getTimestamp()
        gyroTs = gyroValues.getTimestamp()
        
        # Publish Accelerometer Data
        redis_client.set('accel_x', acceleroValues.x)
        redis_client.set('accel_y', acceleroValues.y)
        redis_client.set('accel_z', acceleroValues.z)

        # TODO: Publish Accel and Gyro Timestamps

        # Publish Gyroscope Data
        redis_client.set('gyro_x', gyroValues.x)
        redis_client.set('gyro_y', gyroValues.y)
        redis_client.set('gyro_z', gyroValues.z)