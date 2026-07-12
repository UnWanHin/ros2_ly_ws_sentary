# Merged per-unit state published by behavior_tree.

uint8 AREA_UNKNOWN=0
uint8 AREA_MY_BASE=1
uint8 AREA_MY_HIGHLAND=2
uint8 AREA_MY_ROADLAND=3
uint8 AREA_ENEMY_BASE=4
uint8 AREA_ENEMY_HIGHLAND=5
uint8 AREA_ENEMY_ROADLAND=6
uint8 AREA_CENTRAL=7
uint8 AREA_MY_PRE_ROADLAND=8
uint8 AREA_ENEMY_PRE_ROADLAND=9

uint8 car_id
uint16 hp
bool has_hp
bool hp_fresh
builtin_interfaces/Time hp_stamp

int16 position_x
int16 position_y
bool has_position
bool position_fresh
builtin_interfaces/Time position_stamp
string position_source

uint8 area_id
string area_name
bool area_used_nearest_fallback
