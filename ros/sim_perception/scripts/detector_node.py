#!/usr/bin/env python3
"""detector: left image_raw -> YOLOX ONNX (onnxruntime) -> treemapper_msgs/Detection2DArray.

Params
  ~model_path        (required) ONNX file shipped with the existing detector
  ~image_topic       /cam/bottom/left/half/image_raw
  ~output_topic      /object_detection/detections
  ~input_size        [416, 416]   (h, w) the model was exported with
  ~input_channels    3            grey is replicated; 1 for a single-channel model
  ~decoded           true         model output is already grid-decoded; false -> decode here (strides 8/16/32)
  ~score_threshold   0.3
  ~nms_threshold     0.45
  ~class_ids         [0]          tree classes
  ~device            cuda | cpu   cuda falls back to CPU inside onnxruntime when the CUDA provider is unavailable
Fails loudly (logfatal + exit 1) when onnxruntime is not installed or the model file does not exist.
Boxes are (left, top, width, height) in input-image pixels; header = the image header. Cost on ``~timing``.
"""
import os
import sys
import time

import numpy as np
import rospy
from sensor_msgs.msg import Image
from treemapper_msgs.msg import Detection2D, Detection2DArray, StageTiming

from sim_perception import yolox
from sim_perception.image_utils import msg_to_grey


def die(fmt, *args):
    rospy.logfatal(fmt, *args)
    sys.exit(1)


class DetectorNode:
    def __init__(self):
        model_path = rospy.get_param("~model_path", "")
        self.input_size = tuple(int(v) for v in rospy.get_param("~input_size", [416, 416]))
        self.channels = int(rospy.get_param("~input_channels", 3))
        self.decoded = bool(rospy.get_param("~decoded", True))
        self.score_thr = float(rospy.get_param("~score_threshold", 0.3))
        self.nms_thr = float(rospy.get_param("~nms_threshold", 0.45))
        self.class_ids = [int(c) for c in rospy.get_param("~class_ids", [0])]
        device = str(rospy.get_param("~device", "cuda")).lower()
        if device not in ("cuda", "cpu"):
            die("detector: ~device must be cuda or cpu, got %r", device)
        if len(self.input_size) != 2 or self.channels not in (1, 3):
            die("detector: bad ~input_size %s / ~input_channels %d", self.input_size, self.channels)

        try:
            import onnxruntime as ort
        except ImportError as e:
            die("detector: onnxruntime is not installed (%s); pip install onnxruntime-gpu (or onnxruntime)", e)
        if not model_path:
            die("detector: ~model_path is required (the ONNX file shipped with the existing detector)")
        if not os.path.isfile(model_path):
            die("detector: model file does not exist: %s", model_path)
        providers = ["CPUExecutionProvider"]
        if device == "cuda":
            providers.insert(0, "CUDAExecutionProvider")
        try:
            self.session = ort.InferenceSession(model_path, providers=providers)
        except Exception as e:  # onnxruntime raises its own exception types
            die("detector: cannot load %s: %s", model_path, e)
        self.input_name = self.session.get_inputs()[0].name
        rospy.loginfo("detector: %s, providers %s, input %s %s", model_path, self.session.get_providers(),
                      self.input_name, self.input_size)

        self.pub = rospy.Publisher(rospy.get_param("~output_topic", "/object_detection/detections"),
                                   Detection2DArray, queue_size=5)
        self.pub_timing = rospy.Publisher("~timing", StageTiming, queue_size=5)
        # queue_size 1 + large buff_size: always work on the newest frame, never build a backlog behind inference
        self.sub = rospy.Subscriber(rospy.get_param("~image_topic", "/cam/bottom/left/half/image_raw"), Image,
                                    self.on_image, queue_size=1, buff_size=2 ** 26)

    def on_image(self, msg):
        t0 = time.perf_counter()
        try:
            grey = msg_to_grey(msg)
        except ValueError as e:
            rospy.logwarn_throttle(5.0, "detector: %s", e)
            return
        tensor, ratio = yolox.preprocess(grey, self.input_size, self.channels)
        raw = self.session.run(None, {self.input_name: tensor})[0]
        pred = raw if self.decoded else yolox.decode(raw, self.input_size)
        boxes = yolox.postprocess(pred, ratio, self.score_thr, self.nms_thr, self.class_ids)

        out = Detection2DArray()
        out.header = msg.header
        for x, y, w, h, score, label in boxes:
            d = Detection2D()
            d.x, d.y, d.width, d.height, d.score, d.label = x, y, w, h, score, label
            out.detections.append(d)
        self.pub.publish(out)
        t = StageTiming()
        t.header = msg.header
        t.stages = ["detector"]
        t.ms = [(time.perf_counter() - t0) * 1000.0]
        self.pub_timing.publish(t)


if __name__ == "__main__":
    rospy.init_node("detector")
    DetectorNode()
    rospy.spin()
