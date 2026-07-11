# Semantic view of RM2026 V2.0 referee 0x0308 custom_info_t.
# user_data_utf16 contains exactly 30 raw UTF-16 bytes in referee wire byte order.

std_msgs/Header header
uint16 sender_id
uint16 receiver_id
uint8[30] user_data_utf16
