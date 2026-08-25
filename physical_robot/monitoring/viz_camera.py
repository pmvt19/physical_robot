import cv2

from physical_robot.robot import Robot

robot = Robot(connection="client")

while True:
    rgb_numpy, depth_numpy = robot.read_rgb_camera()

    cv2.imshow("frame", rgb_numpy)
    cv2.imshow("depth frame", depth_numpy)

    if cv2.waitKey(1) == ord("q"):
        break
