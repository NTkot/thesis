std_msgs/Header header

# GPS Timestamp
GpsTime gps_time

# Longitude/Langitude in degrees with decimal points indicating minutes/seconds
# Sign of value indicates:
#   Longitude: North -> Positive, South -> Negative
#   Latitude:  East  -> Positive, West  -> Negative
float64 longitude_deg
float64 latitude_deg

# Character variables indicating direction
# Longitude: 'E'/'W'
# Latitude:  'N'/'S'
char longitude_dir
char LONGITUDE_EAST = 69 # ('E')
char LONGITUDE_WEST = 87 # ('W')
char latitude_dir
char LATITUDE_NORTH = 78 # ('N')
char LATITUDE_SOUTH = 83 # ('S')

# Ground speed in km/h
float32 ground_speed_kmh

# Track angle in deg
float32 track_deg

# Magnetic variation variables:
# - Easterly means subtract magvar_deg from track_deg for the correct course
# - Westerly means add magvar_deg to track_deg for the correct course
float32 mag_var_deg
char mag_var_dir
char MAG_VAR_EAST = 69 # ('E')
char MAG_VAR_WEST = 87 # ('W')

# Position status
bool pos_valid
