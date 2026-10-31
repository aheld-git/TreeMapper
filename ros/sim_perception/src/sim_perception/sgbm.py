"""OpenCV semi-global block matching, configured to spec section 7."""
import numpy as np
import cv2

_MODES = {
    "sgbm": cv2.STEREO_SGBM_MODE_SGBM,
    "sgbm_3way": cv2.STEREO_SGBM_MODE_SGBM_3WAY,
    "hh": cv2.STEREO_SGBM_MODE_HH,
}

_DEFAULTS = {
    "min_disparity": 0,
    "num_disparities": 64,
    "block_size": 5,
    "uniqueness": 10,
    "speckle_window": 100,
    "speckle_range": 2,
    "disp12_max_diff": 1,
    "pre_filter_cap": 31,
    "mode": "sgbm_3way",
    "p1": None,   # None -> 8 * block_size^2
    "p2": None,   # None -> 32 * block_size^2
}


def make_matcher(params):
    """cv2.StereoSGBM from a (possibly empty) parameter dict; unspecified keys take the spec-7 defaults."""
    unknown = set(params) - set(_DEFAULTS)
    if unknown:
        raise ValueError("unknown sgbm parameter(s): %s" % ", ".join(sorted(unknown)))
    p = dict(_DEFAULTS)
    p.update(params)
    nd, bs = int(p["num_disparities"]), int(p["block_size"])
    if nd <= 0 or nd % 16:
        raise ValueError("num_disparities must be a positive multiple of 16, got %d" % nd)
    if bs < 1 or bs % 2 == 0:
        raise ValueError("block_size must be odd and >= 1, got %d" % bs)
    mode = str(p["mode"]).lower()
    if mode not in _MODES:
        raise ValueError("unknown sgbm mode %r (want one of %s)" % (p["mode"], sorted(_MODES)))
    p1 = 8 * bs * bs if p["p1"] is None else int(p["p1"])
    p2 = 32 * bs * bs if p["p2"] is None else int(p["p2"])
    return cv2.StereoSGBM_create(
        minDisparity=int(p["min_disparity"]), numDisparities=nd, blockSize=bs, P1=p1, P2=p2,
        disp12MaxDiff=int(p["disp12_max_diff"]), preFilterCap=int(p["pre_filter_cap"]),
        uniquenessRatio=int(p["uniqueness"]), speckleWindowSize=int(p["speckle_window"]),
        speckleRange=int(p["speckle_range"]), mode=_MODES[mode])


def disparity(left_grey, right_grey, matcher):
    """Disparity in pixels (float32), NaN where SGBM found no match. OpenCV returns 16x fixed point, invalid <= 0."""
    raw = matcher.compute(left_grey, right_grey)
    disp = raw.astype(np.float32) / 16.0
    disp[disp <= 0.0] = np.nan
    return disp
