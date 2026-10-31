"""SGBM wrapper: parameters, known-shift recovery, invalid -> NaN, and the msg -> grey conversion."""
import numpy as np
import cv2
import pytest

from sim_perception import sgbm
from sim_perception.image_utils import to_grey


def _textured(h=120, w=240, seed=3):
    rng = np.random.RandomState(seed)
    img = rng.randint(0, 256, size=(h, w)).astype(np.uint8)
    return cv2.GaussianBlur(img, (3, 3), 0)  # random but not pixel-white noise


def test_recovers_known_shift():
    left = _textured()
    shift = 12
    # rectified pair: a left pixel at x appears at x - d in the right image
    right = np.roll(left, -shift, axis=1)
    disp = sgbm.disparity(left, right, sgbm.make_matcher({}))
    assert disp.dtype == np.float32 and disp.shape == left.shape
    valid_region = disp[10:-10, 80:-20]  # left of x = num_disparities has no match range; wrap-around columns excluded
    finite = valid_region[np.isfinite(valid_region)]
    assert finite.size > 0.5 * valid_region.size
    assert abs(float(np.median(finite)) - shift) <= 0.5
    # invalid pixels (OpenCV: <= 0 after /16) are NaN, never 0 or negative
    assert not np.any(disp[np.isfinite(disp)] <= 0.0)
    assert np.isnan(disp[:, :60]).all()  # columns without a full disparity search range are invalid


def test_invalid_is_nan_not_negative():
    flat = np.full((64, 128), 100, np.uint8)  # no texture: SGBM cannot match anything
    disp = sgbm.disparity(flat, flat, sgbm.make_matcher({}))
    assert np.isnan(disp).any()
    assert not np.any(disp[np.isfinite(disp)] <= 0.0)


def test_defaults_are_spec_values():
    m = sgbm.make_matcher({})
    assert m.getNumDisparities() == 64
    assert m.getBlockSize() == 5
    assert m.getUniquenessRatio() == 10
    assert m.getSpeckleWindowSize() == 100
    assert m.getSpeckleRange() == 2
    assert m.getP1() == 8 * 5 * 5
    assert m.getP2() == 32 * 5 * 5
    assert m.getMode() == cv2.STEREO_SGBM_MODE_SGBM_3WAY


def test_p1_p2_follow_block_size_and_overrides():
    m = sgbm.make_matcher({"block_size": 7})
    assert (m.getP1(), m.getP2()) == (8 * 49, 32 * 49)
    m = sgbm.make_matcher({"block_size": 7, "p1": 11, "p2": 99})
    assert (m.getP1(), m.getP2()) == (11, 99)


def test_bad_params_rejected():
    with pytest.raises(ValueError):
        sgbm.make_matcher({"num_disparities": 60})  # must be a positive multiple of 16
    with pytest.raises(ValueError):
        sgbm.make_matcher({"block_size": 4})  # must be odd
    with pytest.raises(ValueError):
        sgbm.make_matcher({"mode": "nope"})
    with pytest.raises(ValueError):
        sgbm.make_matcher({"bogus_key": 1})


def test_to_grey_accepts_isaac_rgb_and_grey_recordings():
    rgb = np.zeros((2, 3, 3), np.uint8)
    rgb[..., 0] = 255  # pure red
    g = to_grey(rgb.tobytes(), 2, 3, 9, "rgb8")
    assert g.shape == (2, 3) and g.dtype == np.uint8
    assert np.all(g == cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY))
    bgr = rgb[..., ::-1].copy()
    assert np.all(to_grey(bgr.tobytes(), 2, 3, 9, "bgr8") == g)
    mono = np.arange(6, dtype=np.uint8).reshape(2, 3)
    assert np.all(to_grey(mono.tobytes(), 2, 3, 3, "mono8") == mono)
    # a row stride larger than the width (padded rows) is honoured
    padded = np.zeros((2, 8), np.uint8)
    padded[:, :3] = mono
    assert np.all(to_grey(padded.tobytes(), 2, 3, 8, "mono8") == mono)
    with pytest.raises(ValueError):
        to_grey(b"\0" * 24, 2, 3, 12, "32FC1")
    with pytest.raises(ValueError):
        to_grey(b"\0" * 5, 2, 3, 3, "mono8")  # truncated buffer
