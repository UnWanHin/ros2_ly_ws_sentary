# Semantic decode of referee 0x020D sentry_info / sentry_info_2 / sentry_info_3.
# Transport source: lower-machine TypeID=7 SentryData plus TypeID=10 sentry_info_3 shadow.

std_msgs/Header header

uint32 sentry_info_raw
uint16 sentry_info_2_raw
uint64 sentry_info_3_raw
uint16 reserved

# 0x020D sentry_info, bit0-31
uint16 exchanged_projectile_allowance
uint8 remote_projectile_exchange_count
uint8 remote_hp_exchange_count
bool can_confirm_free_revive
bool can_exchange_immediate_revive
uint16 immediate_revive_cost
bool sentry_info_reserved

# 0x020D sentry_info_2, bit0-15
bool out_of_combat
uint16 remaining_exchangeable_17mm
uint8 posture
bool can_activate_energy_mechanism
bool enhanced_posture
bool sentry_info_2_reserved # compatibility alias for bit15

# 0x020D sentry_info_3, bit0-63
bool has_sentry_info_3
uint32 sentry_info_3_age_ms # TypeID=10 age when this message was published; UINT32_MAX means unavailable
uint8 attack_posture_remaining_s
uint8 defense_posture_remaining_s
uint8 move_posture_remaining_s
uint8 sentry_info_3_reserved_low
uint8 enhanced_attack_posture_remaining_s
uint8 enhanced_defense_posture_remaining_s
uint8 enhanced_move_posture_remaining_s
uint8 sentry_info_3_reserved_high
