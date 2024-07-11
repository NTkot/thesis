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
