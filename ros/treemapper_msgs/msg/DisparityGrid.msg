# RECORDING ONLY (sim_perception/disparity_grid_node.py, scripts/sim_mission.sh --record-frames): the pixels of ONE
# disparity image that treemapper_node samples, so a bag does not have to hold the ~1 MB image. treemapper_node itself
# keeps consuming the full sensor_msgs/Image.
#
# data = the raw float32 values at image rows r*stride + stride/2, columns c*stride + stride/2 (row-major, r < grid_height,
# c < grid_width), NaN / zero / negative left as they are: validity is decided by the reader (sim_frames), exactly as
# treemapper_node does it. The src_* fields are the source image's own header fields, so the reader can apply the node's
# malformed / wrong-size rules to them.
# A source image that treemapper_node would call malformed (encoding != 32FC1, big endian, zero size, step != 4 * width,
# data size != step * height) is published too, with grid_width = grid_height = 0 and empty data.
Header header                     # copied from the disparity image (stamp = the left image's stamp)
uint32 src_width
uint32 src_height
uint32 src_step
string src_encoding
uint8 src_is_bigendian
uint32 src_data_size              # bytes in the source image's data
uint32 stride                     # sampling stride in source pixels (treemapper_node: disparity_stride, 4)
uint32 grid_width
uint32 grid_height
float32[] data                    # grid_height * grid_width values, or empty when the source was malformed
