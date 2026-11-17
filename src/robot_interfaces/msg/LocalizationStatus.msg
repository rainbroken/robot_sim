std_msgs/Header header

uint8 UNINITIALIZED=0
uint8 LOCALIZING=1
uint8 TRACKING=2
uint8 LOST=3

uint8 state
string state_label
string map_id
bool map_loaded
bool pose_valid
bool lidar_data_fresh
float64 lidar_data_age_sec
bool lidar_timestamp_valid
float64 lidar_timestamp_age_sec
bool initial_pose_received
float64 registration_error
float64 inlier_ratio
float64 condition_number
uint32 num_inliers
uint32 consecutive_successes
uint32 consecutive_failures
string message
