"""YOLOX pre/post-processing in pure numpy (no onnxruntime needed)."""
import numpy as np
import pytest

from sim_perception import yolox


def test_letterbox_ratio_and_pad():
    img = np.full((100, 200), 7, np.uint8)  # grey, h=100 w=200
    tensor, ratio = yolox.preprocess(img, (416, 416), 3)
    assert ratio == pytest.approx(min(416 / 100, 416 / 200)) == pytest.approx(2.08)
    assert tensor.shape == (1, 3, 416, 416) and tensor.dtype == np.float32
    # resized content: 208 rows x 416 cols at the top-left, 114 below; no normalisation
    assert np.all(tensor[0, :, :208, :] == 7.0)
    assert np.all(tensor[0, :, 208:, :] == 114.0)
    # grey replicated to every channel
    assert np.array_equal(tensor[0, 0], tensor[0, 2])


def test_letterbox_tall_image_pads_right_and_channel_handling():
    img = np.zeros((300, 100, 3), np.uint8)
    img[...] = (10, 20, 30)
    tensor, ratio = yolox.preprocess(img, (416, 416), 3)
    assert ratio == pytest.approx(416 / 300)
    new_w = int(100 * ratio)
    assert np.all(tensor[0, :, :, new_w:] == 114.0)
    assert tensor[0, 0, 5, 5] == 10 and tensor[0, 2, 5, 5] == 30  # colour order kept
    t1, _ = yolox.preprocess(img, (416, 416), 1)
    assert t1.shape == (1, 1, 416, 416)
    with pytest.raises(ValueError):
        yolox.preprocess(img, (416, 416), 2)


def test_decode_grid():
    # input 32x32, stride 8 only -> a 4x4 grid, 16 anchors, row-major (y then x)
    raw = np.zeros((1, 16, 6), np.float32)  # cx, cy, w, h, obj, cls0 all raw 0
    raw[0, 3 * 4 + 2, 4] = 0.9  # mark the anchor at grid cell x=2, y=3
    out = yolox.decode(raw, (32, 32), strides=(8,))
    assert out.shape == raw.shape
    a = out[0, 3 * 4 + 2]
    assert a[0] == pytest.approx((2 + 0) * 8)   # cx = (grid_x + 0) * stride = 16
    assert a[1] == pytest.approx((3 + 0) * 8)   # cy = 24
    assert a[2] == pytest.approx(np.exp(0) * 8)  # w = 8
    assert a[3] == pytest.approx(8.0)
    assert a[4] == pytest.approx(0.9)            # objectness / class columns untouched
    # non-zero offsets follow (grid + xy) * stride and exp(wh) * stride
    raw[0, 0, :4] = (0.5, 0.25, np.log(2.0), np.log(3.0))
    out = yolox.decode(raw, (32, 32), strides=(8,))
    assert out[0, 0, 0] == pytest.approx(0.5 * 8) and out[0, 0, 1] == pytest.approx(0.25 * 8)
    assert out[0, 0, 2] == pytest.approx(16.0) and out[0, 0, 3] == pytest.approx(24.0)
    # input is not modified
    assert raw[0, 3 * 4 + 2, 0] == 0.0


def test_decode_three_strides_and_size_mismatch():
    n = (416 // 8) ** 2 + (416 // 16) ** 2 + (416 // 32) ** 2
    raw = np.zeros((n, 6), np.float32)  # 2-D input (no batch axis) is accepted
    out = yolox.decode(raw, (416, 416))
    assert out.shape == raw.shape
    assert out[0, 2] == pytest.approx(8.0)           # first anchor stride 8
    assert out[(416 // 8) ** 2, 2] == pytest.approx(16.0)  # first stride-16 anchor
    assert out[-1, 2] == pytest.approx(32.0)
    with pytest.raises(ValueError):
        yolox.decode(raw[:-1], (416, 416))


def _pred(rows):
    """rows: (cx, cy, w, h, obj, cls_score_per_class...)"""
    return np.array(rows, np.float32)


def test_nms_and_scaling():
    ratio = 2.0
    pred = _pred([
        # two near-identical label-0 boxes (shifted by 5 of 100 px -> IoU 0.905): one survives
        [50, 50, 100, 100, 0.9, 1.0, 0.0],
        [55, 50, 100, 100, 0.8, 1.0, 0.0],
        [300, 300, 40, 20, 0.9, 1.0, 0.0],   # separate box, kept
        [200, 200, 40, 40, 0.9, 0.0, 1.0],   # label 1: dropped by the class filter
        [400, 100, 30, 30, 0.1, 1.0, 0.0],   # score 0.1 < threshold
    ])
    out = yolox.postprocess(pred, ratio, score_thr=0.3, nms_thr=0.45, class_ids=[0])
    assert len(out) == 2
    out.sort(key=lambda d: -d[4])
    x, y, w, h, score, label = out[0]
    # best box: centre (50, 50) size 100 -> left/top (0, 0), then / ratio -> (0, 0, 50, 50)
    assert (x, y, w, h) == pytest.approx((0.0, 0.0, 50.0, 50.0))
    assert score == pytest.approx(0.9) and label == 0
    x, y, w, h, score, label = out[1]
    assert (x, y, w, h) == pytest.approx(((300 - 20) / 2, (300 - 10) / 2, 20.0, 10.0))
    assert isinstance(label, int)


def test_nms_keeps_boxes_below_iou_threshold():
    pred = _pred([
        [50, 50, 100, 100, 0.9, 1.0],
        [100, 50, 100, 100, 0.8, 1.0],  # shifted 50 -> IoU 1/3 < 0.45: both kept
    ])
    out = yolox.postprocess(pred, 1.0, 0.3, 0.45, [0])
    assert len(out) == 2


def test_nms_overlap_exactly_point_eight():
    # IoU = 0.8: boxes 100 wide, shifted by 100*(1-0.8)/(1+0.8)=11.111 -> inter/union = 0.8
    d = 100 * 0.2 / 1.8
    pred = _pred([[50, 50, 100, 100, 0.9, 1.0], [50 + d, 50, 100, 100, 0.85, 1.0]])
    assert len(yolox.postprocess(pred, 1.0, 0.3, 0.45, [0])) == 1
    assert len(yolox.postprocess(pred, 1.0, 0.3, 0.85, [0])) == 2  # nms_thr above IoU keeps both


def test_score_is_objectness_times_class_and_empty_cases():
    pred = _pred([[50, 50, 10, 10, 0.5, 0.5]])  # 0.25 < 0.3
    assert yolox.postprocess(pred, 1.0, 0.3, 0.45, [0]) == []
    assert yolox.postprocess(np.zeros((0, 6), np.float32), 1.0, 0.3, 0.45, [0]) == []
    # a leading batch axis is accepted
    out = yolox.postprocess(_pred([[50, 50, 10, 10, 0.9, 0.9]])[None], 1.0, 0.3, 0.45, [0])
    assert len(out) == 1 and out[0][4] == pytest.approx(0.81, abs=1e-6)


def test_nms_is_per_class():
    pred = _pred([
        [50, 50, 100, 100, 0.9, 1.0, 0.0],
        [50, 50, 100, 100, 0.8, 0.0, 1.0],  # identical box but another class: kept when both classes wanted
    ])
    assert len(yolox.postprocess(pred, 1.0, 0.3, 0.45, [0, 1])) == 2
    assert len(yolox.postprocess(pred, 1.0, 0.3, 0.45, [1])) == 1
