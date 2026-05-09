import cv2

from physical_robot.models.segmentation.image_segmentation import ImageSegmenter

def segment_live_image_feed():

    cap = cv2.VideoCapture(1)

    image_segmenter = ImageSegmenter()

    while True:
        ret, frame = cap.read()

        cv2.imshow("Raw Frame", frame)

        if cv2.waitKey(1) == ord('q'):
            break
    
    # segment_image


segment_live_image_feed()
    

