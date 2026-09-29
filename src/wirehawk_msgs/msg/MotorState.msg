# Per-motor feedback, one entry per motor (same order as MotorCommand).
int32[]  position     # actual position, encoder counts (0x6064)
int16[]  torque       # actual torque, 0.1% of rated (0x6077)
uint16[] status_word  # CiA 402 status word (0x6041)
uint16[] error_code   # CiA 402 error code (0x603F)
