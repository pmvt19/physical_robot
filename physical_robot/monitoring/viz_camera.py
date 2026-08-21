import cv2
import numpy as np
import redis

redis_client = redis.Redis(host='localhost', port=6379, db=0)

while True:
    rgb_img_bytes = redis_client.get("rgb_img")
    stereo_img_bytes = redis_client.get("stereo_img")

    rgb_img_shape_bytes = redis_client.get("rgb_img_shape")
    stereo_img_shape_bytes = redis_client.get("stereo_img_shape")

    rgb_img_shape = np.frombuffer(rgb_img_shape_bytes, dtype=np.int64)
    stereo_img_shape = np.frombuffer(stereo_img_shape_bytes, dtype=np.int64)

    rgb_img = np.frombuffer(rgb_img_bytes, dtype=np.uint8).reshape(480, 640, 3)
    stereo_img = np.frombuffer(stereo_img_bytes, dtype=np.int16).reshape(240, 320)

    print("rgb", rgb_img.shape)
    print("Stereo", stereo_img.shape)

    cv2.imshow('frame', rgb_img)
    cv2.imshow('depth frame', stereo_img)

    if cv2.waitKey(1) == ord('q'):
        break