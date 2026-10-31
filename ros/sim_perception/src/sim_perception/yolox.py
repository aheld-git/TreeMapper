"""YOLOX pre- and post-processing in numpy (cv2 only for the resize), independent of the inference runtime.

Layout follows the YOLOX reference (`yolox/data/data_augment.py::preproc`, `yolox/models/yolo_head.py::decode_outputs`,
`yolox/utils/demo_utils.py::demo_postprocess`): letterbox into the top-left corner with value 114, raw 0..255 float
input, NCHW; output rows ``(cx, cy, w, h, obj, cls_0 .. cls_n)``, grid decode ``(xy + grid) * stride`` and
``exp(wh) * stride`` over the strides concatenated in order.
"""
from typing import List, Sequence, Tuple

import numpy as np
import cv2

PAD_VALUE = 114


def preprocess(img, input_size, channels):
    """Letterbox ``img`` (HxW grey or HxWx3) into ``input_size`` = (h, w). Returns (1, C, h, w) float32 and the ratio."""
    h_in, w_in = int(input_size[0]), int(input_size[1])
    if channels not in (1, 3):
        raise ValueError("input channels must be 1 or 3, got %r" % (channels,))
    if img.ndim == 2:
        img = img[:, :, None]
    if img.ndim != 3 or img.shape[2] not in (1, 3):
        raise ValueError("image must be HxW or HxWx{1,3}, got shape %s" % (img.shape,))
    h, w = img.shape[:2]
    if h < 1 or w < 1:
        raise ValueError("empty image")
    if img.shape[2] == 3 and channels == 1:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)[:, :, None]
    ratio = min(h_in / h, w_in / w)
    new_w, new_h = max(1, int(w * ratio)), max(1, int(h * ratio))
    resized = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    if resized.ndim == 2:  # cv2 drops a singleton channel axis
        resized = resized[:, :, None]
    if resized.shape[2] != channels:  # grey -> replicate
        resized = np.repeat(resized, channels, axis=2)
    canvas = np.full((h_in, w_in, channels), PAD_VALUE, np.float32)
    canvas[:new_h, :new_w] = resized
    tensor = np.ascontiguousarray(canvas.transpose(2, 0, 1)[None])
    return tensor, ratio


def decode(raw, input_size, strides: Sequence[int] = (8, 16, 32)):
    """Grid-decode raw head output (N, 5+nc) or (1, N, 5+nc) -> same shape, xy centre / wh in input pixels."""
    raw = np.asarray(raw, dtype=np.float32)
    squeeze = raw.ndim == 2
    out = (raw[None] if squeeze else raw).copy()
    h_in, w_in = int(input_size[0]), int(input_size[1])
    grids, strd = [], []
    for s in strides:
        hs, ws = h_in // s, w_in // s
        xv, yv = np.meshgrid(np.arange(ws), np.arange(hs))
        grids.append(np.stack((xv, yv), axis=2).reshape(-1, 2))
        strd.append(np.full((hs * ws, 1), s))
    grid = np.concatenate(grids, axis=0).astype(np.float32)
    stride = np.concatenate(strd, axis=0).astype(np.float32)
    if out.shape[1] != grid.shape[0]:
        raise ValueError("%d anchors in the model output but input %s with strides %s needs %d"
                         % (out.shape[1], tuple(input_size), tuple(strides), grid.shape[0]))
    out[..., 0:2] = (out[..., 0:2] + grid) * stride
    out[..., 2:4] = np.exp(out[..., 2:4]) * stride
    return out[0] if squeeze else out


def _nms(boxes, scores, thr):
    """Greedy NMS on xyxy boxes; returns kept indices, best score first."""
    order = np.argsort(-scores, kind="stable")
    keep = []
    while order.size:
        i = order[0]
        keep.append(int(i))
        if order.size == 1:
            break
        rest = order[1:]
        xx1 = np.maximum(boxes[i, 0], boxes[rest, 0])
        yy1 = np.maximum(boxes[i, 1], boxes[rest, 1])
        xx2 = np.minimum(boxes[i, 2], boxes[rest, 2])
        yy2 = np.minimum(boxes[i, 3], boxes[rest, 3])
        inter = np.maximum(0.0, xx2 - xx1) * np.maximum(0.0, yy2 - yy1)
        area_i = (boxes[i, 2] - boxes[i, 0]) * (boxes[i, 3] - boxes[i, 1])
        area_r = (boxes[rest, 2] - boxes[rest, 0]) * (boxes[rest, 3] - boxes[rest, 1])
        iou = inter / np.maximum(area_i + area_r - inter, 1e-9)
        order = rest[iou <= thr]
    return keep


def postprocess(pred, ratio, score_thr, nms_thr, class_ids) -> List[Tuple[float, float, float, float, float, int]]:
    """Decoded rows (cx, cy, w, h, obj, cls...) -> [(x, y, w, h, score, label)] in input-image pixels.

    score = obj * best class score; label = argmax class; rows below ``score_thr`` or with a label outside
    ``class_ids`` are dropped; greedy per-class NMS at ``nms_thr`` (suppress when IoU > nms_thr); boxes are divided by
    ``ratio`` to undo the letterbox. (x, y) is the left/top corner.
    """
    pred = np.asarray(pred, dtype=np.float32)
    if pred.ndim == 3:
        pred = pred[0]
    if pred.size == 0:
        return []
    cls = pred[:, 5:]
    labels = np.argmax(cls, axis=1)
    scores = pred[:, 4] * cls[np.arange(len(pred)), labels]
    mask = (scores >= score_thr) & np.isin(labels, np.asarray(list(class_ids), dtype=labels.dtype))
    if not mask.any():
        return []
    pred, labels, scores = pred[mask], labels[mask], scores[mask]
    xyxy = np.empty((len(pred), 4), np.float32)
    xyxy[:, 0] = pred[:, 0] - pred[:, 2] / 2
    xyxy[:, 1] = pred[:, 1] - pred[:, 3] / 2
    xyxy[:, 2] = pred[:, 0] + pred[:, 2] / 2
    xyxy[:, 3] = pred[:, 1] + pred[:, 3] / 2
    kept = []
    for c in np.unique(labels):
        idx = np.nonzero(labels == c)[0]
        kept.extend(int(idx[k]) for k in _nms(xyxy[idx], scores[idx], nms_thr))
    kept.sort(key=lambda i: -scores[i])
    out = []
    for i in kept:
        x1, y1, x2, y2 = (xyxy[i] / ratio).tolist()
        out.append((x1, y1, x2 - x1, y2 - y1, float(scores[i]), int(labels[i])))
    return out
