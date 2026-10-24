# All detections of one image. header.stamp MUST equal the image stamp it was computed on.
# (The existing detector publishes spot_spraying_msgs/DetectionResult with the same fields;
#  the input node adapts it, so either can be fed.)
Header header
Detection2D[] detections
