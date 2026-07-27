# Command codes for the overloaded legacy-compatible `type` field.
int64 SET_CURRENT_POINT=-1
int64 SET_PATH_POINT_A=0
int64 SET_PATH_POINT_B=1
int64 SET_PATH_POINT_C=2
int64 SET_DRAW_OFFSET=3
# Values 4 and 5 are retired legacy commands.
int64 DRAW_RECTANGLE=6
int64 DRAW_TRIANGLE=7
int64 DRAW_CIRCLE=8

# Cartesian values in millimetres. Their meaning depends on `type`.
float32 x0
float32 y0
float32 z0
int64 type
