std_msgs/Header header

# Integer variable showing antenna used/status. Check values below
int8 antenna_status
int8 INTERNAL_ANTENNA_USED    = 1
int8 EXTERNAL_ANTENNA_USED    = 2
int8 EXTERNAL_ANTENNA_SHORTED = 3
int8 ANTENNA_USED_ERROR       = 4
