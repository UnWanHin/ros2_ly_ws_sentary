# Semantic control velocity for the lower-machine 1-byte X/Y velocity fields.
# If use_raw is true, raw_x/raw_y are written to the serial frame directly.
# Otherwise gimbal_driver encodes x_mps/y_mps using io_config/velocity_raw_to_mps.

std_msgs/Header header
float32 x_mps
float32 y_mps
int8 raw_x
int8 raw_y
bool use_raw
