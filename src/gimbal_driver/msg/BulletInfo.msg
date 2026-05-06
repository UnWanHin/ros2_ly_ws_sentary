# Semantic bullet/referee resource status from lower-machine TypeID=7/8.
# TypeID=7 provides 0x0207 initial_speed; TypeID=8 provides 0x0207
# bullet_type/shooter_number/launching_frequency, 0x0208 projectile_allowance,
# and 0x0209 rfid_status_2.

std_msgs/Header header

bool has_initial_speed
float32 initial_speed

bool has_shoot_data
uint8 bullet_type
uint8 shooter_number
uint8 launching_frequency

bool has_projectile_allowance
uint16 projectile_allowance_17mm
uint16 projectile_allowance_42mm
uint16 remaining_gold_coin
uint16 projectile_allowance_fortress_17mm

bool has_rfid_status_2
uint8 rfid_status_2_raw
