import depthai as dai
import numpy as np
import redis

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
        print("Running Pipeline")
        while pipeline.isRunning():
            colorFrame : dai.ImgFrame = colorOut.get()
            stereoFrame : dai.ImgFrame = stereoOut.get()

            rgb_img : np.ndarray = colorFrame.getFrame()
            stereo_img : np.ndarray = stereoFrame.getFrame()

            print("Before computing")
            rgb_byte_img = rgb_img.tobytes()
            stereo_byte_img = stereo_img.tobytes()
            print("After computing")

            print("Publishing", rgb_img.shape, stereo_img.shape)
            # redis_client.set('rgb_img', rgb_img.tobytes())
            # redis_client.set('stereo_img', stereo_img.tobytes())
            # redis_client.set('rgb_img', rgb_img)
            # redis_client.set('stereo_img', stereo_img)
            # redis_client.set('time', time.time())
            # redis_client.set('info', 'hello')
            print("Done publishing")
            

    except KeyboardInterrupt:
        print("\nCtrl+C detected. Performing cleanup...")
        print("Cleanup complete. Exiting.")
    
    pipeline.stop()