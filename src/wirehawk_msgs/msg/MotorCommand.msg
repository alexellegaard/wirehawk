# Target motor positions, one entry per motor (same order as MotorState).
# Units: raw encoder counts. The bridge is a dumb motor bridge — it does not
# know about cables, spools, or the CDPR geometry.
int32[] position
