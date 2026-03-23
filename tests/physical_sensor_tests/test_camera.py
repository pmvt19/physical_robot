import depthai as dai
import numpy as np
import cv2

with dai.Pipeline() as pipeline:
    color = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    monoLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
    stereo = pipeline.create(dai.node.StereoDepth)

    imu = pipeline.create(dai.node.IMU)

    # enable ACCELEROMETER_RAW at 500 hz rate
    imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, 480)

    # enable GYROSCOPE_RAW at 400 hz rate
    imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, 400)

    imu.setBatchReportThreshold(1)
    imu.setMaxBatchReports(10)



    # stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DETAIL)
    # stereo.setOutputSize(640, 480)
    # stereo.depth.setOutputSize(640, 480)
    # stereo.
    # colorCamOut = color.requestOutput((640, 480))
    # monoLeftOut = monoLeft.requestOutput((640, 480))
    # monoRightOut = monoRight.requestOutput((640, 480))
    resolution = (640, 480)
    colorCamOut = color.requestOutput(resolution)
    monoLeftOut = monoLeft.requestOutput(resolution)
    monoRightOut = monoRight.requestOutput(resolution)

    # stereo.depth

    monoLeftOut.link(stereo.left)
    monoRightOut.link(stereo.right)

    colorOut = colorCamOut.createOutputQueue(maxSize=1, blocking=False)
    leftOut = monoLeftOut.createOutputQueue(maxSize=1, blocking=False)
    rightOut = monoRightOut.createOutputQueue(maxSize=1, blocking=False)
    stereoOut = stereo.depth.createOutputQueue(maxSize=1, blocking=False)
    disparityOut = stereo.disparity.createOutputQueue(maxSize=1, blocking=False)

    imuOut = imu.out.createOutputQueue(maxSize=1, blocking=False)

    pipeline.start()
    try:
        print("Running Pipeline")
        while pipeline.isRunning():
            colorFrame : dai.ImgFrame = colorOut.get()
            stereoFrame : dai.ImgFrame = stereoOut.get()

            leftFrame : dai.ImgFrame = leftOut.get()
            rightFrame : dai.ImgFrame = rightOut.get()

            disparityFrame : dai.ImgFrame = disparityOut.get()

            rgb_img : np.ndarray = colorFrame.getFrame()
            stereo_img : np.ndarray = stereoFrame.getFrame()

            rgb_img : np.ndarray = colorFrame.getCvFrame()
            stereo_img : np.ndarray = stereoFrame.getCvFrame()

            left_img : np.ndarray = leftFrame.getCvFrame()
            right_img : np.ndarray = rightFrame.getCvFrame()

            disparity_img : np.ndarray = disparityFrame.getCvFrame()

            print(rgb_img.shape, stereo_img.shape, left_img.shape, right_img.shape, disparity_img.shape)

            # print("Publishing")
            cv2.imshow('rgb img', rgb_img)
            cv2.imshow('depth img', stereo_img)

            cv2.imshow("left_img", left_img)
            cv2.imshow("right_img", right_img)

            cv2.imshow("disparity_img", disparity_img)

            imuData = imuOut.get()
            imuPacket = imuData.packets[0]
            acceleroValues = imuPacket.acceleroMeter
            gyroValues = imuPacket.gyroscope

            acceleroTs = acceleroValues.getTimestamp()
            gyroTs = gyroValues.getTimestamp()

            # print(f"Accelerometer Values: {acceleroValues.x}, {acceleroValues.y}, {acceleroValues.z}")
            # print(f"Gyroscope Values: {gyroValues.x}, {gyroValues.y}, {gyroValues.z}")


            if cv2.waitKey(1) == ord('q'):
                break

    except KeyboardInterrupt:
        print("\nCtrl+C detected. Performing cleanup...")
        print("Cleanup complete. Exiting.")