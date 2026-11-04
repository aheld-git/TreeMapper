"""disparity_grid: the decimated pixels are exactly the ones treemapper_node samples (input_sync.cpp sampleDisparity)."""
import numpy as np
import pytest

from sim_perception import disparity_grid as dg


def full_image(w=640, h=400):
    # every pixel unique, so any wrong sample position shows
    return np.arange(w * h, dtype="<f4").reshape(h, w)


def test_grid_size_matches_the_nodes_gridsize():
    for pixels in (640, 400, 7, 3, 2, 1, 0):
        assert dg.grid_size(pixels, 4) == len(range(2, pixels, 4))
    assert dg.grid_size(640, 4) == 160 and dg.grid_size(400, 4) == 100 and dg.grid_size(5, 0) == 0


def test_samples_rows_4r_plus_2_and_cols_4c_plus_2():
    img = full_image()
    f = dg.grid_fields(640, 400, 2560, "32FC1", False, img.tobytes())
    assert (f["grid_width"], f["grid_height"], f["stride"]) == (160, 100, 4)
    grid = np.asarray(f["data"]).reshape(100, 160)
    for r, c in ((0, 0), (0, 159), (99, 0), (99, 159), (37, 81)):
        assert grid[r, c] == img[4 * r + 2, 4 * c + 2]
    assert np.array_equal(grid, img[2::4, 2::4])
    assert (f["src_width"], f["src_height"], f["src_step"], f["src_encoding"], f["src_data_size"]) == \
        (640, 400, 2560, "32FC1", 640 * 400 * 4)


def test_invalid_values_pass_through_untouched():
    img = full_image(16, 8).copy()
    img[2, 2], img[2, 6], img[6, 2] = np.nan, 0.0, -3.0
    grid = np.asarray(dg.grid_fields(16, 8, 64, "32FC1", False, img.tobytes())["data"]).reshape(2, 4)
    assert np.isnan(grid[0, 0]) and grid[0, 1] == 0.0 and grid[1, 0] == -3.0


def test_a_message_is_far_smaller_than_the_image():
    f = dg.grid_fields(640, 400, 2560, "32FC1", False, full_image().tobytes())
    assert f["data"].nbytes == 64000      # 100 x 160 float32, vs 1 024 000 bytes


@pytest.mark.parametrize("kw", [dict(encoding="16UC1"), dict(bigendian=True), dict(width=0), dict(height=0),
                                dict(step=640), dict(data=b"\0" * 100)])
def test_malformed_sources_give_an_empty_grid_but_keep_their_fields(kw):
    a = dict(width=640, height=400, step=2560, encoding="32FC1", bigendian=False, data=full_image().tobytes())
    a.update(kw)
    f = dg.grid_fields(a["width"], a["height"], a["step"], a["encoding"], a["bigendian"], a["data"])
    assert f["grid_width"] == 0 and f["grid_height"] == 0 and len(f["data"]) == 0
    assert f["src_width"] == a["width"] and f["src_step"] == a["step"] and f["src_data_size"] == len(a["data"])
