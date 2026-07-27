# Requested start and target Cartesian coordinates in millimetres.
# main_node owns the authoritative current start pose; xo/yo/zo are retained
# for wire compatibility and diagnostics.
float32 xo
float32 yo
float32 zo
float32 xf
float32 yf
float32 zf

# Reserved legacy field. The simulator currently ignores it.
int64 gripper
