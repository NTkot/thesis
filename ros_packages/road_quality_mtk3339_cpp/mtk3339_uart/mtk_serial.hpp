#ifndef MTK_GPS_HPP
#define MTK_GPS_HPP

#include <cstdio>
#include <string>
#include <ctime>

#include <nmea/gpgll.h>
#include <nmea/gprmc.h>
#include <nmea/gpvtg.h>
#include <nmea/gpgga.h>
#include <nmea/gpgsa.h>
#include <nmea/gpgsv.h>

#include "mtk_utils.hpp"

#define NMEA_BUFSIZ 512


namespace mtk_3339
{
    class MTK3339Serial 
    {
        private:
            FILE* _fp;

            char recv_buffer[NMEA_BUFSIZ];
            char parse_buffer[NMEA_BUFSIZ];
            int buff_pos;
            bool nmea_received;

            int _custom_parse();
            int _set_port_raw();
            int _set_port_baud(baud_t baud);
            std::string _print_time(const struct tm &date);
        
        public:
            settings_t settings;

            nmea_gpgll_s gpgll;
            nmea_gprmc_s gprmc;
            nmea_gpvtg_s gpvtg;
            nmea_gpgga_s gpgga;
            nmea_gpgsa_s gpgsa;
            nmea_gpgsv_s gpgsv;
            nmea_pcd11_t pcd11;

            MTK3339Serial(settings_t user_settings);

            /**
             * @brief Initialize the UART communication.
             * @return Returns 0 for success, negative number for error:
             *         -1 for error setting the baud rate, 
             *         -2 for error setting the update rate, 
             *         -3 for error setting GPS outputs, 
             *         -4 for error setting antenna output, 
             *         -5 for error enabling/disabling SBAS, 
             *         -6 for error setting SBAS mode
             */
            int init();

            /**
             * @brief Send a command to the MTK3339 GPS module over UART.
             * 
             * @param cmd The command string to send.
             * @return Returns 0 for success, otherwise an error code.
             *         For error codes, check output of `fprintf()` method.
             */
            int send_cmd(std::string cmd);

            /**
             * @brief This function reads a character from the UART interface using `fgetc`.
             *        Character read is stored in the reception buffer.  
             * 
             * @return The read character as an integer, or a negative value if an error occurs.
             */
            int read_char();

            /**
             * @brief Parses reception buffer to extract NMEA messages 
             *        and stores the data in the appropriate member 
             *        variables, based on message type.
             *
             * @return int:
             * - 1: Successfully parsed a GPGLL message.
             * - 2: Successfully parsed a GPRMC message.
             * - 3: Successfully parsed a GPVTG message.
             * - 4: Successfully parsed a GPGGA message.
             * - 5: Successfully parsed a GPGSA message.
             * - 6: Successfully parsed a GPGSV message.
             * - 7: Successfully parsed a PCD11 message.
             * - 8: Successfully parsed a PMTKx message.
             * - -1: Buffer overflow, message too large to parse.
             * - -2: Failed to parse the NMEA message.
             * - -3: Unknown NMEA message type.
             * - -5: No NMEA message received.
             */
            int parse_nmea();

            /* Functions used to set options to sensor */
            int set_baud_rate(baud_t baud);
            int set_update_rate(update_rate_t update_ms);
            int set_gps_outputs(gps_output_t gps_out);
            int set_antenna_output(antenna_output_t antenna_output);
            int set_sbas_enabled(sbas_enabled_t sbas_enabled);
            int set_sbas_mode(sbas_mode_t sbas_mode);

            /* Functions to convert NMEA sentences to strings */
            std::string gpgll_to_str();
            std::string gprmc_to_str();
            std::string gpvtg_to_str();
            std::string gpgga_to_str();
            std::string gpgsa_to_str();
            std::string gpgsv_to_str();
            std::string pcd11_to_str();
    };
}

#endif 