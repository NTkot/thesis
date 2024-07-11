std_msgs/Header header

# Total number of sentences
uint8 n_sentences

# Current sentence
uint8 current_sentence

# Number of satellites in view. May be different than the number of satellites in use (see GPGGA)
uint8 n_satellites_view

# Satellites in-view information arrays
int16[4] sat_prn            # Satellite PRN number:
                            #   GPS     = 1 to 32
                            #   Galileo = 1 to 36
                            #   BeiDou  = 1 to 63
                            #   NavIC   = 1 to 14
                            #   QZSS    = 1 to 10
                            #   SBAS    = 33 to 64 (add 87 for PRN#s)
                            #   GLONASS = 65 to 96
int16[4] sat_elevation_deg  # Elevation, degrees, 90 maximum
int16[4] sat_azimuth_deg    # Azimuth, degrees True, 000 to 359
int16[4] sat_snr            # SNR (C/No) 00-99 dB, null when not tracking
