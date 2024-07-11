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

# Antenna altitude above/below mean sea level, and its respective unit
float32 altitude
char altitude_unit

# Undulation - the relationship between the geoid and the WGS84 ellipsoid
float32 undulation
char undulation_unit

# Number of satellites in use. May be different to the number in-view (see GPGSV)
uint8 n_satellites

# Position fix type
uint8 position_fix_type
