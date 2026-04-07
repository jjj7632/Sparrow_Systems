# -*- coding: utf-8 -*-
"""
Minimal SoC side image holder.
The full image history service is expected to live in MATLAB
"""

import time

import numpy as np


class LatestFrameStore(object):
    # Initialize an empty slot for the newest frame seen by the SoC
    def __init__(self):
        self.frame_number = None
        self.image_data = None
        self.timestamp = None

    # Replace the current frame snapshot with new image data
    def put(self, frame_number, image_data):
        self.frame_number = frame_number
        self.image_data = image_data
        self.timestamp = time.time()

    # Return the newest stored frame and its capture timestamp
    def get(self):
        if self.image_data is None:
            return None
        return {
            "frame_number": self.frame_number,
            "image_data": self.image_data,
            "timestamp": self.timestamp
        }

    # Drop any stored frame data from the cache
    def clear(self):
        self.frame_number = None
        self.image_data = None
        self.timestamp = None

    # Report whether a frame has been stored yet
    def is_empty(self):
        return self.image_data is None


def create_dummy_image(width, height, value, channels=1):
    if width <= 0 or height <= 0:
        raise ValueError("width and height must be positive")
    if channels <= 0:
        raise ValueError("channels must be positive")
    if value < 0 or value > 255:
        raise ValueError("value must be 0-255")

    if channels == 1:
        return np.full((height, width), value, dtype=np.uint8)

    return np.full((height, width, channels), value, dtype=np.uint8)
