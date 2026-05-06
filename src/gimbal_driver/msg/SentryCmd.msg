# Semantic view of referee 0x0301 / data_cmd_id=0x0120 sentry_cmd.
# field_mask is for partial command updates on /ly/referee/sentry_cmd:
# - 0 means this message is a complete snapshot and all fields are applied.
# - otherwise set FIELD_* bits for fields carried by this message.

uint8 FIELD_CONFIRM_FREE_REVIVE=1
uint8 FIELD_CONFIRM_IMMEDIATE_REVIVE=2
uint8 FIELD_EXCHANGE_PROJECTILE_ALLOWANCE=4
uint8 FIELD_REMOTE_PROJECTILE_EXCHANGE_COUNT=8
uint8 FIELD_REMOTE_HP_EXCHANGE_COUNT=16
uint8 FIELD_POSTURE=32
uint8 FIELD_CONFIRM_ENERGY_ACTIVATE=64
uint8 FIELD_ALL=127

std_msgs/Header header
uint8 field_mask
bool confirm_free_revive
bool confirm_immediate_revive
uint16 exchange_projectile_allowance
uint8 remote_projectile_exchange_count
uint8 remote_hp_exchange_count
uint8 posture
bool confirm_energy_activate
uint32 raw
