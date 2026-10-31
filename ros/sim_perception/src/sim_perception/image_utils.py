"""sensor_msgs/Image data -> 8-bit grey, without cv_bridge (so it is testable without ROS).

Isaac publishes rgb8 ``image_raw``; the real recordings are mono8. Both nodes work on grey, so either input is accepted.
"""
import numpy as np
import cv2

# encoding -> (channels, cv2 colour conversion code to grey or None when already grey)
_ENCODINGS = {
    "mono8": (1, None),
    "8UC1": (1, None),
    "rgb8": (3, cv2.COLOR_RGB2GRAY),
    "bgr8": (3, cv2.COLOR_BGR2GRAY),
    "rgba8": (4, cv2.COLOR_RGBA2GRAY),
    "bgra8": (4, cv2.COLOR_BGRA2GRAY),
}


def to_grey(data, height, width, step, encoding):
    """Return an (height, width) uint8 grey image from raw image bytes (honours the row stride ``step``)."""
    if encoding not in _ENCODINGS:
        raise ValueError("unsupported image encoding %r (want one of %s)" % (encoding, sorted(_ENCODINGS)))
    channels, code = _ENCODINGS[encoding]
    if height <= 0 or width <= 0 or step < width * channels or len(data) < height * step:
        raise ValueError("malformed image: %dx%d step %d, %d bytes, encoding %s"
                         % (width, height, step, len(data), encoding))
    buf = np.frombuffer(data, dtype=np.uint8, count=height * step).reshape(height, step)
    img = np.ascontiguousarray(buf[:, :width * channels])
    if channels == 1:
        return img.reshape(height, width)
    return cv2.cvtColor(img.reshape(height, width, channels), code)


def msg_to_grey(msg):
    """to_grey for a sensor_msgs/Image."""
    return to_grey(msg.data, msg.height, msg.width, msg.step, msg.encoding)
