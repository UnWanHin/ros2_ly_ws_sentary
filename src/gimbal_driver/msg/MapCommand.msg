# Semantic view of referee 0x0303 map_command_t.
# Transport source: lower-machine TypeID=9 MapCommandData.
# 0x0303 repeats duplicate packets; consumers must deduplicate before actions.

std_msgs/Header header

bool has_target_position
float32 target_position_x_m
float32 target_position_y_m

bool has_target_robot
uint8 target_robot_id

uint8 cmd_keyboard
uint16 cmd_source
