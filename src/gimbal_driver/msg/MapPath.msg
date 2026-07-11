# Semantic view of RM2026 V2.0 referee 0x0307 map_data_t.
# Coordinates and deltas use dm. The 49 delta pairs describe points after start.

std_msgs/Header header
uint8 intention
uint16 start_position_x_dm
uint16 start_position_y_dm
int8[49] delta_x_dm
int8[49] delta_y_dm
uint16 sender_id
