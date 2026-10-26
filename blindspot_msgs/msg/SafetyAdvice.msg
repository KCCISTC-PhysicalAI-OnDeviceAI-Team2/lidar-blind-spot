std_msgs/Header header   # stamp = latest original time among the inputs used, frame_id = "base_jig"
uint8 PASS=0
uint8 SLOW=1
uint8 STOP=2
uint8 level
bool  lidar_hit                       # a LiDAR obstacle was inside the zone
bool  camera_hit                      # a camera obstacle was inside the zone
builtin_interfaces/Time lidar_stamp   # stamp of the LiDAR message used (0 if not used)
builtin_interfaces/Time camera_stamp  # stamp of the camera message used (0 if not used)
string reason                         # e.g. "camera:parcel_box x=1.02 y=0.01 -> STOP"