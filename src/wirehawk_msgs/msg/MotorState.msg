# Per-motor feedback, one entry per motor (same order as MotorCommand).
# Units are FREE-SPAN cable length [m]: anchor (pulley) -> TCP. The bridge owns
# the raw-encoder -> free-span conversion (spool model + zero + mast_run), so the
# controller never sees encoder counts.
#
# CHANGING THIS FILE REQUIRES BUMPING SCHEMA_VERSION in BOTH:
#   wirehawk_ethercat_bridge/bridge.hpp::kSchemaVersion  AND  cdpr_node.py::SCHEMA_VERSION
# (and rebuilding wirehawk_msgs on the Pi, not just the bridge).
float32[] position     # measured free-span length, anchor (pulley) -> TCP [m]
float32[] target       # commanded free-span the interpolator is driving the drum toward [m]
int16[]   torque       # actual torque, 0.1% of rated (0x6077)
uint16[]  status_word  # CiA 402 status word (0x6041)
uint16[]  error_code   # CiA 402 error code (0x603F)
