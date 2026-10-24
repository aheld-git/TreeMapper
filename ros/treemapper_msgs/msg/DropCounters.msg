# Rule 7: every dropped data point is counted by reason, per stage. Counts are cumulative.
Header header
string stage                      # e.g. "association", "stereo", "fusion"
string[] reasons                  # e.g. "outside_support", "above_drone", "beyond_range"
uint64[] counts                   # same length as reasons
