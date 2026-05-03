import numpy as np
import threading
import math
from utils import point_segment_distance

class ThreadWithReturn(threading.Thread):
    def __init__(self, group=None, target=None, name=None, args=(), kwargs={}, verbose=None):
        super().__init__(group, target, name, args, kwargs)
        self._return = None
    
    def run(self):
        if self._target is not None:
            self._return = self._target(*self._args, **self._kwargs)
    
    def join(self):
        super().join()
        return self._return


def parallel_point_segment_distance(segments: np.ndarray, points: np.ndarray, batch_size=1000) -> np.ndarray:
    threads = []
    num_points = len(points)
    num_batches = math.ceil(num_points / batch_size)

    for batch in range(num_batches):
        start_idx = batch * batch_size
        end_idx = min((batch+1) * batch_size, num_points)

        sub_batch_points = points[start_idx:end_idx]
        t = ThreadWithReturn(target=point_segment_distance, args=(segments, sub_batch_points, ))
        threads.append(t)
        t.start()

    results = []
    for t in threads:
        results.append(t.join())

    return np.concatenate(results, axis=1)