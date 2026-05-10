import cv2

from physical_robot.models.segmentation.image_segmentation import ImageSegmenter

def segment_live_image_feed():

    cap = cv2.VideoCapture(0)

    image_segmenter = ImageSegmenter()

    while True:
        # Read Frame from Camera
        ret, frame = cap.read()

        # Segment Image
        prediction, _ = image_segmenter.segment_image(frame)
        prediction_frame = prediction['segmentation'].numpy()

        cv2.imshow("Raw Frame", frame)
        cv2.imshow("Prediction", prediction_frame / prediction_frame.max())

        if cv2.waitKey(1) == ord('q'):
            break

segment_live_image_feed()
