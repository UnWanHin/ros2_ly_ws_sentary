std_msgs/Header header

uint8 DIRECTION_RX=0
uint8 DIRECTION_TX=1
uint8 TYPE_ID_TX_CONTROL=255
uint8 TYPE_ID_TX_SENTRY_COMMAND=254
uint8 TYPE_ID_TX_MAP_PATH=253
uint8 TYPE_ID_TX_CUSTOM_INFO=252
uint8 TYPE_ID_TX_SENTRY_COORDINATE=251
uint8 TYPE_ID_TX_TRAJECTORY=250

uint8 direction
uint8 type_id
uint8[] data

uint8 firecode_raw
uint32 sentry_cmd_raw
