# Runtime health/status for the FaceMode solver chain.

std_msgs/Header header

# True only when FaceMode has target + gimbal feedback + TF + solved command
# and is publishing an angle command.
bool function

bool has_target
builtin_interfaces/Time target_stamp
float32 target_age_sec
uint32 target_update_count
bool has_gimbal_angles
bool gimbal_stamp_ok
bool raw_goal_static_calibration_ready
bool manual_target

bool tf_target_to_solve_frame
bool tf_target_to_aim_frame
bool tf_target_to_camera_frame
bool tf_target_to_gimbal_barrel_joint
bool tf_target_to_gimbal_barrel

bool solver_ok
bool publishing_angles
builtin_interfaces/Time last_success_stamp
float32 last_success_age_sec

string failed_stage
string detail

string active_target_frame
geometry_msgs/Point active_target

string solve_mode
string solve_frame
string aim_frame
string camera_frame

float32 current_yaw
float32 current_pitch
float32 command_yaw
float32 command_pitch
