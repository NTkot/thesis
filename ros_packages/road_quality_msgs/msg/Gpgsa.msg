std_msgs/Header header

# A = Automatic 2D/3D
# M = Manual, forced to operate in 2D or 3D
char mode
char MODE_AUTOMATIC = 65 # ('A')
char MODE_MANUAL    = 77 # ('M')

# Fix type: 1 = no fix, 2 = 2D (SV < 4), 3 = 3D (SV >= 4)
uint8 fix_type
uint8 FIX_TYPE_UNDEFINED     = 0
uint8 FIX_TYPE_NOT_AVAILABLE = 1
uint8 FIX_TYPE_2D            = 2
uint8 FIX_TYPE_3D            = 3

# PRN numbers of satellites used in solution (null for unused fields), total of 12 fields
# GPS     = 1 to 32
# Galileo = 1 to 36
# BeiDou  = 1 to 63
# NavIC   = 1 to 14
# QZSS    = 1 to 10
# SBAS    = 33 to 64 (add 87 for PRN#s)
# GLONASS = 65 to 96
int32[12] sat_ids

# Dilutions of precision
float32 pdop    # Position
float32 hdop    # Horizontal
float32 vdop    # Vertical
