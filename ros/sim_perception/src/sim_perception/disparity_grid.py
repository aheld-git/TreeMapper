"""The pixels treemapper_node samples from one disparity image (input_sync.cpp ``sampleDisparity``), as plain fields.

No ROS imports: ``grid_fields`` takes the header fields of a ``sensor_msgs/Image`` and returns the fields of a
``treemapper_msgs/DisparityGrid`` (``data`` = the raw float32 values on the grid, row ``r*stride + stride//2``, column
``c*stride + stride//2``). The values are NOT filtered (NaN, zero and negative stay): the reader applies the validity
rule. ``malformed`` mirrors the node's ``DisparityStatus::kMalformed``.
"""
import numpy as np

ENCODING = "32FC1"


def grid_size(pixels: int, stride: int) -> int:
    """Number of grid samples along an axis (``gridSize`` in input_sync.cpp)."""
    if pixels <= 0 or stride <= 0:
        return 0
    o = stride // 2
    return (pixels - o + stride - 1) // stride if pixels > o else 0


def is_malformed(encoding: str, width: int, height: int, step: int, bigendian: bool, data_size: int) -> bool:
    return (encoding != ENCODING or bool(bigendian) or width == 0 or height == 0 or step != 4 * width
            or data_size != step * height)


def grid_fields(width: int, height: int, step: int, encoding: str, bigendian: bool, data: bytes, stride: int = 4) -> dict:
    """Fields of the DisparityGrid for one image (everything but ``header``)."""
    out = {"src_width": int(width), "src_height": int(height), "src_step": int(step), "src_encoding": str(encoding),
           "src_is_bigendian": int(bool(bigendian)), "src_data_size": len(data), "stride": int(stride),
           "grid_width": 0, "grid_height": 0, "data": []}
    if stride <= 0 or is_malformed(encoding, width, height, step, bigendian, len(data)):
        return out
    o = stride // 2
    img = np.frombuffer(data, dtype="<f4").reshape(height, width)
    grid = np.ascontiguousarray(img[o::stride, o::stride])
    out["grid_height"], out["grid_width"] = int(grid.shape[0]), int(grid.shape[1])
    out["data"] = grid.ravel()
    return out
