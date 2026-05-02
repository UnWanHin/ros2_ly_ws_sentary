# Semantic view of the 1-byte lower-machine FireCode field.
# field_mask is for partial command updates on /ly/control/firecode:
# - 0 means this message is a complete snapshot and all fields are applied.
# - otherwise set FIELD_* bits for fields carried by this message.
# Feedback on /ly/gimbal/firecode always uses FIELD_ALL and fills raw.

uint8 FIELD_FIRE_STATUS=1
uint8 FIELD_CAP_STATE=2
uint8 FIELD_FOLLOW_MODE=4
uint8 FIELD_AIM_MODE=8
uint8 FIELD_ROTATE=16
uint8 FIELD_ALL=31

std_msgs/Header header
uint8 field_mask
uint8 fire_status
uint8 cap_state
bool follow_mode
bool aim_mode
uint8 rotate
uint8 raw
