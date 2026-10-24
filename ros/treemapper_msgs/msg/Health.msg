# Heartbeat and health. Published at a fixed rate even while asleep ("sleeping, not dead").
uint8 AWAKE=0
uint8 SLEEPING=1                  # deliberately idle (e.g. spraying, idle, pre-mission)
uint8 HOLDING=2                   # awake but holding processing (e.g. no valid pose yet)
uint8 FAULT=3

Header header
uint8 state
string reason                     # human-readable why (e.g. "phase SPRAYING", "pose not valid")
MissionPhase phase                # phase and target in force
uint32 trees_published
float64 last_input_age_s          # age of the newest camera/detection input
string[] warnings                 # active warnings (fail-open gates that triggered, clamps, ...)
