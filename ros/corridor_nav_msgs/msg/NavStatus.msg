# Status of the corridor navigator (published on ~status at 2 Hz).
Header header
string state                  # nav state (IDLE TAKEOFF TRANSIT SCAN_OUT SCAN_SETTLE LOCK GLIDE DOSE RETURN LAND HOLD ABORT DONE, or CLEANUP)
int32 corridor                # current corridor index, -1 = none
int32 corridors_total
int32 target_id               # TreeMapper ID, -1 = none
uint32 trees_sprayed
uint32 trees_skipped
uint32 trees_remaining        # in locked corridors + cleanup list
float64 clearance_m           # latest nadir canopy clearance from stereo, NaN if invalid
string[] reasons              # counter names (holds, guard triggers, climbs, retries, skips, re-resolved IDs, ...)
uint64[] counts               # cumulative, same length as reasons
string[] warnings             # active warnings
