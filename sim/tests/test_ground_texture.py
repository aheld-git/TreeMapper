import numpy as np
import pytest
from isaac import ground_texture as gt

RGB = (0.30, 0.42, 0.18)


def luma(img):
    return img.astype(float) @ np.array([0.299, 0.587, 0.114])


def test_deterministic_shape_and_hue():
    a = gt.grass_texture(RGB, seed=3, size=256)
    b = gt.grass_texture(RGB, seed=3, size=256)
    c = gt.grass_texture(RGB, seed=4, size=256)
    assert a.shape == (256, 256, 3) and a.dtype == np.uint8
    assert np.array_equal(a, b) and not np.array_equal(a, c)
    # brightness modulation only: the mean colour stays the terrain colour, green stays the largest channel
    assert np.allclose(a.reshape(-1, 3).mean(axis=0) / 255.0, RGB, atol=0.02)
    assert (a[..., 1] >= a[..., 0]).mean() > 0.99


def test_has_texture_at_the_camera_footprint():
    # the stereo matcher needs contrast at the scale one pixel covers: 3.3 cm at 15 m (fx 454.5), 1.2 cm at 5.5 m.
    # Average the tile down to 3.3 cm cells (8 m / 256): neighbouring cells must still differ clearly.
    img = luma(gt.grass_texture(RGB, seed=1))                  # 1024 px over 8 m
    cells = img.reshape(256, 4, 256, 4).mean(axis=(1, 3))      # 3.1 cm cells
    grad = np.abs(np.diff(cells, axis=1)).mean()
    assert cells.std() > 0.08 * cells.mean()                   # >= 8 % brightness spread
    assert grad > 0.04 * cells.mean()                          # and it is fine-scale, not only slow shading
    flat = luma(np.full((1024, 1024, 3), 128, np.uint8)).reshape(256, 4, 256, 4).mean(axis=(1, 3))
    assert np.abs(np.diff(flat, axis=1)).mean() == 0.0


def test_tiles_seamlessly():
    img = luma(gt.grass_texture(RGB, seed=2, size=512))
    inner = np.abs(np.diff(img, axis=1)).mean()
    wrap = np.abs(img[:, 0] - img[:, -1]).mean()
    assert wrap < 1.5 * inner                                  # the edge pair looks like any neighbour pair
    inner_v = np.abs(np.diff(img, axis=0)).mean()
    assert np.abs(img[0] - img[-1]).mean() < 1.5 * inner_v


def test_uv_is_planar_world_xy_over_the_tile():
    uv = gt.ground_uv([(0.0, 0.0, 1.0), (8.0, -4.0, 0.5), (-17.0, 17.0, 0.0)], tile_m=8.0)
    assert np.allclose(uv, [(0.0, 0.0), (1.0, -0.5), (-2.125, 2.125)])


def test_bad_parameters_raise():
    with pytest.raises(ValueError):
        gt.grass_texture(RGB, size=8)
    with pytest.raises(ValueError):
        gt.grass_texture(RGB, wavelength_m=(0.5, 0.1))
