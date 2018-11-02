std_msgs/Header header
string          device_type
string          device_name
string          channel
bool            ros_sync_enabled
string          time_unit
int64           sensor_timestamp      # Device Timestamp @ half the exposure time. Closest to "capture time"
int64           frame_timestamp       # Device Timestamp @ frame transmission starts
int64           backend_timestamp     # PC Timestamp @ kernel receives frame information
int64           arrival_timestamp     # PC Timestamp @ user receives frame information
int64           publish_timestamp     # PC Timestamp @ ROS driver publishes image
int64           device_fps
bool            auto_exposure
int64           exposure_time
int64           frame_number
