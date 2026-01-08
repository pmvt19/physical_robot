import numpy as np
from PIL import Image
import torch
import cv2
from collections import defaultdict
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib import cm
import matplotlib

from transformers import AutoImageProcessor, Mask2FormerForUniversalSegmentation

# TODO: Look into using CUDA for faster inference if available

class ImageSegmenter():
    def __init__(self, model="facebook/mask2former-swin-base-coco-panoptic"):
        self.processor = AutoImageProcessor.from_pretrained(model)
        self.model = Mask2FormerForUniversalSegmentation.from_pretrained(model)

    def _get_all_segment_labels(self, prediction):
        segmentation_labels = {
            segment['id'] : self.model.config.id2label[segment['label_id']].split('-')[0] for segment in prediction['segments_info']
        }
        return segmentation_labels

    def segment_image(self, image : np.ndarray):
        image = Image.fromarray(image)
        inputs = self.processor(image, return_tensors="pt")

        with torch.no_grad():
            outputs = self.model(**inputs)

        prediction = self.processor.post_process_panoptic_segmentation(outputs, target_sizes=[image.size[::-1]])[0]
        return prediction, self._get_all_segment_labels(prediction)

    def draw_panoptic_segmentation(self, ax, segmentation, segments_info):
        # TODO: Improve the labeling of this function
        # get the used color map
        viridis = matplotlib.colormaps["Set1"]
        ax.imshow(segmentation)
        # instances_counter = defaultdict(int)
        handles = []
        # for each segment, draw its legend
        for segment in segments_info:
            segment_id = segment['id']
            segment_label_id = segment['label_id']
            segment_label = self.model.config.id2label[segment_label_id]
            print(segment_id, segment_label)
            # label = f"{segment_label}-{instances_counter[segment_label_id]}"
            # instances_counter[segment_label_id] += 1
            color = viridis(segment_id)
            
        ax.legend(handles=handles)
        plt.show()

    def get_instance_segment_mask(self, segmentation, segments_info, prompt):
        for segment in segments_info:
            segment_label_id = segment['label_id']
            segment_label = self.model.config.id2label[segment_label_id]
            if (prompt in segment_label.lower()):
                mask = (segmentation == segment['id']).cpu().numpy()
                return mask
        return segmentation == -1  # Return an empty mask if not found
    
    # TODO: Determine if needed?
    def batch_get_instance_segment_masks(self, segmentation, segments_info, prompts):
        mask = segmentation == -1  # Start with an empty mask
        for prompt in prompts:
            mask = np.logical_or(mask, self.get_instance_segment_mask(segmentation, segments_info, prompt))
        return mask
    
if __name__ == '__main__':
    import time
    # Test on Webcam
    capture = cv2.VideoCapture(1)
    time.sleep(1)

    image_segmenter = ImageSegmenter()

    while True:
        ret, frame = capture.read()

        prediction, labels = image_segmenter.segment_image(frame)
        print(labels)
        fig, ax = plt.subplots(1, 2)
        ax[0].imshow(frame[:, :, ::-1])
        ax[1].imshow(prediction['segmentation'])
        plt.show()