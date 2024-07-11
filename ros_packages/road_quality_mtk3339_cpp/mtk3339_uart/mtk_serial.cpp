#include <iostream>
#include <stdexcept>
#include <cstdlib>
#include <cstring>
#include <sstream>
#include <chrono>
#include <thread>
#include <ctime>

#include <nmea.h>
#include <nmea/gpgll.h>
#include <nmea/gprmc.h>
#include <nmea/gpvtg.h>
#include <nmea/gpgga.h>
#include <nmea/gpgsa.h>
#include <nmea/gpgsv.h>

#include "mtk_serial.hpp"
#include "mtk_utils.hpp"

namespace mtk_3339
{
    MTK3339Serial::MTK3339Serial(settings_t user_settings) : settings(user_settings),
                                                             buff_pos(-1),
                                                             nmea_received(false)
    {
        int ret = this->_set_port_raw();
        if(ret != 0)
        {
            throw std::runtime_error("Could not configure serial port");
        }

        this->_fp = fopen(this->settings.device.c_str(), "rw+");
        if(this->_fp == NULL)
        {
            throw std::runtime_error("Could not open serial port");
        }
    }


    int MTK3339Serial::init()
    {
        if(this->set_baud_rate(this->settings.baud_rate) != 0)
            return -1;

        if(this->set_update_rate(this->settings.update_ms) != 0)
            return -2;

        if(this->set_gps_outputs(this->settings.gps_out) != 0)
            return -3;

        if(this->set_antenna_output(this->settings.antenna_output) != 0)
            return -4;

        if(this->set_sbas_enabled(this->settings.sbas_enabled) != 0)
            return -5;

        if(this->set_sbas_mode(this->settings.sbas_mode) != 0)
            return -6;

        return 0;
    }


    int MTK3339Serial::send_cmd(std::string cmd)
    {
        // std::cout << "Sending command: " << cmd << std::endl;
        int ret = fprintf(this->_fp, "\r\n%s\r\n", cmd.c_str());
        return ret;
    }


    int MTK3339Serial::read_char()
    {
        int ret = fgetc(this->_fp);
        if(ret < 0)
            return ret;
        char c = (char)ret;

        // Check if start of NMEA sentence
        if(c == '$')
        {
            this->buff_pos = 0;
            this->nmea_received = false;
        }
        // Check if end of NMEA sentence
        else if(c == '\n')
        {
            this->nmea_received = true;
        }
        
        // Store character to buffer
        if(this->buff_pos > -1)
        {
            this->recv_buffer[this->buff_pos++] = c;
        }

        return ret;
    }


    int MTK3339Serial::parse_nmea()
    {
        if(this->nmea_received)
        {
            if(this->buff_pos > (NMEA_BUFSIZ - 1))
            {
                return -1;
            }

            memcpy(this->parse_buffer, this->recv_buffer, this->buff_pos);
            this->parse_buffer[this->buff_pos] = '\0';
            // std::cout << "Received message: " << this->parse_buffer << std::endl;

            if(this->parse_buffer[1] == 'G' && this->parse_buffer[2] == 'P')
            {
                // $GPXXX NMEA message
                nmea_s* data;
                data = nmea_parse(this->parse_buffer, strlen(this->parse_buffer), 1);

                if(data == NULL)
                {
                    return -2;
                }
                else if(data->type == NMEA_GPGLL)
                {
                    gpgll = *((nmea_gpgll_s *) data);
                    nmea_free(data);
                    return 1;
                }
                else if(data->type == NMEA_GPRMC)
                {
                    gprmc = *((nmea_gprmc_s *) data);
                    nmea_free(data);
                    return 2;
                }
                else if(data->type == NMEA_GPVTG)
                {
                    gpvtg = *((nmea_gpvtg_s *) data);
                    nmea_free(data);
                    return 3;
                }
                else if(data->type == NMEA_GPGGA)
                {
                    gpgga = *((nmea_gpgga_s *) data);
                    nmea_free(data);
                    return 4;
                }
                else if(data->type == NMEA_GPGSA)
                {
                    gpgsa = *((nmea_gpgsa_s *) data);
                    nmea_free(data);
                    return 5;
                }
                else if(data->type == NMEA_GPGSV)
                {
                    gpgsv = *((nmea_gpgsv_s *) data);
                    nmea_free(data);
                    return 6;
                }
                else
                {
                    return -3;
                }
            }
            else
            {
                return this->_custom_parse();
            }
        }
        
        return -5;
    }


    int MTK3339Serial::set_baud_rate(baud_t baud)
    {
        std::string cmd = baud_to_nmea(baud);
        int ret = this->send_cmd(cmd);
        if(ret < 0)
        {
            return -1;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(250));

        int conf_ret = this->_set_port_baud(baud);
        if(conf_ret != 0)
        {
            return -2;
        }
        this->buff_pos = -1;
        this->nmea_received = false;

        return 0;
    }


    int MTK3339Serial::set_update_rate(update_rate_t update_ms)
    {
        std::string cmd = update_rate_to_nmea(update_ms);
        int ret = this->send_cmd(cmd);
        if(ret < 0)
        {
            return -1;
        }
        return 0;
    }


    int MTK3339Serial::set_gps_outputs(gps_output_t gps_out)
    {
        std::string cmd = gps_output_to_nmea(gps_out);
        int ret = this->send_cmd(cmd);
        if(ret < 0)
        {
            return -1;
        }
        return 0;
    }


    int MTK3339Serial::set_antenna_output(antenna_output_t antenna_output)
    {
        std::string cmd = antenna_output_to_nmea(antenna_output);
        int ret = this->send_cmd(cmd);
        if(ret < 0)
        {
            return -1;
        }
        return 0;
    }


    int MTK3339Serial::set_sbas_enabled(sbas_enabled_t sbas_enabled)
    {
        std::string cmd = sbas_enabled_to_nmea(sbas_enabled);
        int ret = this->send_cmd(cmd);
        if(ret < 0)
        {
            return -1;
        }
        return 0;
    }


    int MTK3339Serial::set_sbas_mode(sbas_mode_t sbas_mode)
    {
        std::string cmd = sbas_mode_to_nmea(sbas_mode);
        int ret = this->send_cmd(cmd);
        if(ret < 0)
        {
            return -1;
        }
        return 0;
    }


    int MTK3339Serial::_custom_parse()
    {
        if(parse_buffer[1] == 'P' && parse_buffer[2] == 'C' &&
        parse_buffer[3] == 'D')
        {
            // $PCD NMEA messages
            if(parse_buffer[4] == ',' && parse_buffer[5] == '1' &&
            parse_buffer[6] == '1' && parse_buffer[7] == ',')
            {
                if(parse_buffer[8] == '1')
                {
                    pcd11 = INTERNAL_ANTENNA_USED;
                }
                else if(parse_buffer[8] == '2')
                {
                    pcd11 = EXTERNAL_ANTENNA_USED;
                }
                else if(parse_buffer[8] == '3')
                {
                    pcd11 = EXTERNAL_ANTENNA_SHORTED;
                }
                else
                {
                    pcd11 = ANTENNA_USED_ERROR;
                }
                return 7;
            }
            
        }
        else if(parse_buffer[1] == 'P' && parse_buffer[2] == 'M' &&
                parse_buffer[3] == 'T' && parse_buffer[4] == 'K')
        {
            // $PMTK NMEA messages
            return 8;
        }

        return -4;
    }


    int MTK3339Serial::_set_port_raw()
    {
        std::string cmd = "stty -F " + this->settings.device + " raw";
        int ret_sys = system(cmd.c_str());
        return ret_sys;
    }


    int MTK3339Serial::_set_port_baud(baud_t baud)
    {
        std::string cmd = "stty -F " + this->settings.device + " " + baud_to_str(baud);
        int ret_sys = system(cmd.c_str());
        return ret_sys;
    }


    std::string MTK3339Serial::_print_time(const struct tm &date)
    {
        return std::to_string(date.tm_mday)        + "/" +
               std::to_string(date.tm_mon + 1)     + "/" +
               std::to_string(date.tm_year + 1900) + " " +
               std::to_string(date.tm_hour)        + ":" +
               std::to_string(date.tm_min)         + ":" +
               std::to_string(date.tm_sec);
    }


    std::string MTK3339Serial::gpgll_to_str()
    {
        std::stringstream str_out;
        str_out << "GPGLL:\n" <<
                   "  Time: " << this->_print_time(this->gpgll.time) << std::endl <<
                   "  Longitude:\n    Degrees:  " << this->gpgll.longitude.degrees << std::endl <<
                                 "    Minutes:  " << this->gpgll.longitude.minutes << std::endl <<
                                 "    Cardinal: " << this->gpgll.longitude.cardinal << std::endl <<
                   "  Latitude:\n    Degrees:  "  << this->gpgll.latitude.degrees << std::endl <<
                                "    Minutes:  "  << this->gpgll.latitude.minutes << std::endl <<
                                "    Cardinal: "  << this->gpgll.latitude.cardinal << std::endl;
        return str_out.str();
    }


    std::string MTK3339Serial::gprmc_to_str()
    {
        std::stringstream str_out;
        str_out << "GPRMC:\n" <<
                   "  Valid: " << ((this->gprmc.valid) ? "True" : "False") << std::endl <<
                   "  Time:  " << this->_print_time(this->gprmc.date_time) << std::endl <<
                   "  Longitude:\n    Degrees:  " << this->gprmc.longitude.degrees << std::endl <<
                                 "    Minutes:  " << this->gprmc.longitude.minutes << std::endl <<
                                 "    Cardinal: " << this->gprmc.longitude.cardinal << std::endl <<
                   "  Latitude:\n    Degrees:  "  << this->gprmc.latitude.degrees << std::endl <<
                                "    Minutes:  "  << this->gprmc.latitude.minutes << std::endl <<
                                "    Cardinal: "  << this->gprmc.latitude.cardinal << std::endl <<
                   "  Speed:\n    Ground speed (knots): " << this->gprmc.gndspd_knots << std::endl <<
                             "    Track (degrees):      " << this->gprmc.track_deg << std::endl <<
                             "    Magnetic Var. deg:    " << this->gprmc.magvar_deg << std::endl <<
                             "    Magnetic Var. car:    " << this->gprmc.magvar_cardinal << std::endl;
        return str_out.str();
    }


    std::string MTK3339Serial::gpvtg_to_str()
    {
        std::stringstream str_out;
        str_out << "GPVTG:\n" <<
                   "  Speed:\n    Ground speed (knots): " << this->gpvtg.gndspd_knots << std::endl <<
                             "    Ground speed (km/h):  " << this->gpvtg.gndspd_kmph << std::endl <<
                             "    Track (degrees):      " << this->gpvtg.track_deg << std::endl;
        return str_out.str();
    }


    std::string MTK3339Serial::gpgga_to_str()
    {
        std::stringstream str_out;
        str_out << "GPGGA:\n" <<
                   "  Time:            " << this->_print_time(this->gpgga.time) << std::endl <<
                   "  Position fix:    " << this->gpgga.position_fix << std::endl <<
                   "  Satellites used: " << this->gpgga.n_satellites << std::endl <<
                   "  Longitude:\n    Degrees:  " << this->gpgga.longitude.degrees << std::endl <<
                                 "    Minutes:  " << this->gpgga.longitude.minutes << std::endl <<
                                 "    Cardinal: " << this->gpgga.longitude.cardinal << std::endl <<
                   "  Latitude:\n    Degrees:  "  << this->gpgga.latitude.degrees << std::endl <<
                                "    Minutes:  "  << this->gpgga.latitude.minutes << std::endl <<
                                "    Cardinal: "  << this->gpgga.latitude.cardinal << std::endl <<
                   "  Altitude:\n    Above sea-level: " << this->gpgga.altitude << std::endl <<
                                "    Unit:            " << this->gpgga.altitude_unit << std::endl << 
                   "  Undulation:\n    Value: " << this->gpgga.undulation << std::endl <<
                                  "    Unit:  " << this->gpgga.undulation_unit << std::endl;
        return str_out.str();
    }


    std::string MTK3339Serial::gpgsa_to_str()
    {
        std::stringstream str_out;
        str_out << "GPGSA:\n" <<
                   "  Mode:     " << this->gpgsa.mode << std::endl <<
                   "  Fix-type: " << this->gpgsa.fixtype << std::endl <<
                   "  Satellite-IDs:\n    00: " << this->gpgsa.satID_00 << std::endl <<
                                     "    01: " << this->gpgsa.satID_01 << std::endl <<
                                     "    02: " << this->gpgsa.satID_02 << std::endl <<
                                     "    03: " << this->gpgsa.satID_03 << std::endl <<
                                     "    04: " << this->gpgsa.satID_04 << std::endl <<
                                     "    05: " << this->gpgsa.satID_05 << std::endl <<
                                     "    06: " << this->gpgsa.satID_06 << std::endl <<
                                     "    07: " << this->gpgsa.satID_07 << std::endl <<
                                     "    08: " << this->gpgsa.satID_08 << std::endl <<
                                     "    09: " << this->gpgsa.satID_09 << std::endl <<
                                     "    10: " << this->gpgsa.satID_10 << std::endl <<
                                     "    11: " << this->gpgsa.satID_11 << std::endl <<
                   "  Precision dilution:\n    Position:   " << this->gpgsa.pdop << std::endl <<
                                          "    Horizontal: " << this->gpgsa.hdop << std::endl <<
                                          "    Vertical:   " << this->gpgsa.vdop << std::endl;
        return str_out.str();
    }


    std::string MTK3339Serial::gpgsv_to_str()
    {
        std::stringstream str_out;
        str_out << "GPGSV:\n" <<
                   "  Satellites in-view: "     << this->gpgsv.satellites << std::endl <<
                   "  Sentences:\n    Total:  " << this->gpgsv.sentences << std::endl <<
                                 "    Actual: " << this->gpgsv.sentence_number << std::endl <<
                   "  Channels:\n" <<
                   "    Chan1:\n      PRN:             " << this->gpgsv.sat[0].prn << std::endl <<
                               "      SNR (dBHz):      " << this->gpgsv.sat[0].snr << std::endl << 
                               "      Elevation (deg): " << this->gpgsv.sat[0].elevation << std::endl <<
                               "      Azimuth (deg):   " << this->gpgsv.sat[0].azimuth << std::endl <<
                   "    Chan2:\n      PRN:             " << this->gpgsv.sat[1].prn << std::endl <<
                               "      SNR (dBHz):      " << this->gpgsv.sat[1].snr << std::endl << 
                               "      Elevation (deg): " << this->gpgsv.sat[1].elevation << std::endl <<
                               "      Azimuth (deg):   " << this->gpgsv.sat[1].azimuth << std::endl <<
                   "    Chan3:\n      PRN:             " << this->gpgsv.sat[2].prn << std::endl <<
                               "      SNR (dBHz):      " << this->gpgsv.sat[2].snr << std::endl << 
                               "      Elevation (deg): " << this->gpgsv.sat[2].elevation << std::endl <<
                               "      Azimuth (deg):   " << this->gpgsv.sat[2].azimuth << std::endl <<
                   "    Chan4:\n      PRN:             " << this->gpgsv.sat[3].prn << std::endl <<
                               "      SNR (dBHz):      " << this->gpgsv.sat[3].snr << std::endl << 
                               "      Elevation (deg): " << this->gpgsv.sat[3].elevation << std::endl <<
                               "      Azimuth (deg):   " << this->gpgsv.sat[3].azimuth << std::endl;
        return str_out.str();
    }


    std::string MTK3339Serial::pcd11_to_str()
    {
        std::stringstream str_out;
        str_out << "PCD11:\n" << "  Antenna used: " << (int)this->pcd11 << std::endl << std::endl;
        return str_out.str();
    }
}
