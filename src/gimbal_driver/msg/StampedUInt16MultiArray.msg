# UInt16MultiArray-compatible payload with a source timestamp.
# Existing position consumers use data[0:2] as official-map centimeters.
# map_point/map_frame/source_frame are optional debug/source fields for publishers
# that also know the map-frame point.
std_msgs/Header header
std_msgs/MultiArrayLayout layout
uint16[] data
geometry_msgs/Point map_point
string map_frame
string source_frame
