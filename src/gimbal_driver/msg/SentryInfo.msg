# Semantic decode of referee 0x020D sentry_info and sentry_info_2.
# Transport source: lower-machine TypeID=7 SentryData.

std_msgs/Header header

uint32 sentry_info_raw
uint16 sentry_info_2_raw
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
bool sentry_info_2_reserved
