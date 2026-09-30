# Command to N motors, one entry per motor (same order as MotorState).
# Units: raw encoder counts. The bridge is a dumb motor bridge — it does not
# know about cables, spools, or the CDPR geometry.
#
# ctrl_word is RESERVED for the drive lifecycle (enable / fault-reset / estop /
# brake-release). It is not used yet: the bench enables on bridge start and
# resets by power cycle. It lives in the message now so adding the lifecycle
# later is not a wire-breaking change.
std_msgs/Header header     # stamp for command freshness; frame_id unused
uint8 ctrl_word            # RESERVED (0 = default)
int32[] position           # target motor position, encoder counts
