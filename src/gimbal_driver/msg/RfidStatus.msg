# Decoded referee 0x0209 rfid_status, aligned with
# RoboMaster 2026 university series communication protocol V1.3.0 (2026-03-27).
# TypeID=4 carries low 32-bit rfid_status.
# TypeID=8 carries the extra rfid_status_2 byte and is merged into this message.

std_msgs/Header header
uint32 raw
bool friend_base
bool friend_central
bool enemy_central
bool friend_highland
bool enemy_highland
bool friend_flyroad_front
bool friend_flyroad_back
bool enemy_flyroad_front
bool enemy_flyroad_back
bool friend_central_under
bool friend_central_high
bool enemy_central_under
bool enemy_central_high
bool friend_roadland_under
bool friend_roadland_high
bool enemy_roadland_under
bool enemy_roadland_high
bool friend_bastion
bool friend_outpost
bool friend_supply_noremix
bool friend_supply_remix
bool friend_armor
bool enemy_armor
bool central_rmul
bool enemy_bastion
bool enemy_outpost
bool friend_tunnel_roadland_down
bool friend_tunnel_roadland_mid
bool friend_tunnel_roadland_up
bool friend_tunnel_highland_low
bool friend_tunnel_highland_mid
bool friend_tunnel_highland_high
bool has_rfid_status_2
uint8 rfid_status_2_raw
bool enemy_tunnel_roadland_down
bool enemy_tunnel_roadland_mid
bool enemy_tunnel_roadland_up
bool enemy_tunnel_highland_low
bool enemy_tunnel_highland_mid
bool enemy_tunnel_highland_high
uint8 rfid_status_2_reserved
