# Press-and-hold Cartesian jog commands.
uint8 STOP=0
uint8 FORWARD=1
uint8 BACK=2
uint8 LEFT=3
uint8 RIGHT=4
uint8 UP=5
uint8 DOWN=6

uint8 command

# Requested Cartesian jog speed in millimetres per second.
# Ignored when command is STOP.
float32 speed_mm_s
