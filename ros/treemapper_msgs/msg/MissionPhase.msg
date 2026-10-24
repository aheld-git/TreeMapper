# Mission phase as understood by the mapper. Navigation publishes std_msgs/String on
# /mission_state (IDLE, SCAN_APPROACH, SCAN, PLANNING, HOPPING, SPRAYING, RETURN, ERROR);
# the node parses that and echoes this message on its health namespace.
uint8 UNKNOWN=0                   # missing/unrecognised: behave as SCAN (permissive) and warn
uint8 IDLE=1
uint8 SCAN_APPROACH=2
uint8 SCAN=3
uint8 PLANNING=4                  # tree list locked: no new standalone trees
uint8 HOPPING=5                   # tree list locked; during descent only the target may change
uint8 SPRAYING=6                  # only the target may change; heavy processing may sleep
uint8 RETURN=7
uint8 ERROR=8

Header header
uint8 phase
string raw                        # the string received from navigation
int32 target_id                   # current hop target (from /spray_target, -1 = none)
