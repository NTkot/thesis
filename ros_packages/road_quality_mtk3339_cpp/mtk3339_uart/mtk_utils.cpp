#include <stdexcept>
#include <sstream>
#include <string>
#include <iostream>

#include "yaml-cpp/yaml.h"

#include "mtk_utils.hpp"


namespace mtk_3339
{
    std::string baud_to_str(baud_t baud)
    {
        switch(baud)
        {
            case BAUD_4800:
                return "4800";
                break;
            case BAUD_9600:
                return "9600";
                break;
            case BAUD_14400:
                return "14400";
                break;
            case BAUD_19200:
                return "19200";
                break;
            case BAUD_38400:
                return "38400";
                break;
            case BAUD_57600:
                return "57600";
                break;
            case BAUD_115200:
                return "115200";
                break;
            default:
                throw std::runtime_error("Invalid baud rate argument in mtk_3339::baud_to_str()"); 
        }
    }

    std::string baud_to_nmea(baud_t baud)
    {
        std::string cmd = "PMTK251," + baud_to_str(baud);
        return generate_full_cmd(cmd);
    }

    baud_t int_to_baud(int baud)
    {
        switch(baud)
        {
            case 4800:
                return BAUD_4800;
                break;
            case 9600:
                return BAUD_9600;
                break;
            case 14400:
                return BAUD_14400;
                break;
            case 19200:
                return BAUD_19200;
                break;
            case 38400:
                return BAUD_38400;
                break;
            case 57600:
                return BAUD_57600;
                break;
            case 115200:
                return BAUD_115200;
                break;
            default:
                throw std::runtime_error("Invalid baud rate argument in mtk_3339::int_to_baud(): " + std::to_string(baud)); 
        }
    }

    std::string update_rate_to_str(update_rate_t rate)
    {
        switch(rate)
        {
            case UPDATE_RATE_100MS:
                return "100";
                break;
            case UPDATE_RATE_200MS:
                return "200";
                break;
            case UPDATE_RATE_500MS:
                return "500";
                break;
            case UPDATE_RATE_1000MS:
                return "1000";
                break;
            default:
                throw std::runtime_error("Invalid update rate argument in mtk_3339::update_rate_to_nmea()"); 
        }
    }

    std::string update_rate_to_nmea(update_rate_t rate)
    {
        std::string cmd = "PMTK220," + update_rate_to_str(rate);
        return generate_full_cmd(cmd);
    }

    update_rate_t int_to_update_rate(int update_ms)
    {
        switch(update_ms)
        {
            case 100:
                return UPDATE_RATE_100MS;
                break;
            case 200:
                return UPDATE_RATE_200MS;
                break;
            case 500:
                return UPDATE_RATE_500MS;
                break;
            case 1000:
                return UPDATE_RATE_1000MS;
                break;
            default:
                throw std::runtime_error("Invalid update rate argument in mtk_3339::int_to_update_rate()"); 
        }
    }

    gps_output_t::gps_output_t(unsigned gll,
                               unsigned rmc,
                               unsigned vtg,
                               unsigned gga,
                               unsigned gsa,
                               unsigned gsv) : gll(gll),
                                               rmc(rmc),
                                               vtg(vtg),
                                               gga(gga),
                                               gsa(gsa),
                                               gsv(gsv) {};

    std::string gps_output_to_nmea(gps_output_t gps_out)
    {
        std::string cmd = "PMTK314," + std::to_string(gps_out.gll) +
                                 "," + std::to_string(gps_out.rmc) +
                                 "," + std::to_string(gps_out.vtg) +
                                 "," + std::to_string(gps_out.gga) +
                                 "," + std::to_string(gps_out.gsa) +
                                 "," + std::to_string(gps_out.gsv) +
                                 ",0,0,0,0,0,0,0,0,0,0,0,0,0";
        return generate_full_cmd(cmd);
    }

    std::string antenna_output_to_nmea(antenna_output_t antenna_output)
    {
        std::string cmd;
        switch(antenna_output)
        {
            case SEND_ANTENNA:
                cmd = "CDCMD,33,1";
                break;
            case NO_SEND_ANTENNA:
                cmd = "CDCMD,33,0";
                break;
            default:
                throw std::runtime_error("Invalid antenna output argument in mtk_3339::antenna_output_to_nmea()"); 
        }
        return generate_full_cmd(cmd);
    }

    std::string sbas_enabled_to_nmea(sbas_enabled_t sbas_enabled)
    {
        std::string cmd;
        switch(sbas_enabled)
        {
            case SBAS_DISABLED:
                cmd = "PMTK313,0";
                break;
            case SBAS_ENABLED:
                cmd = "PMTK313,1";
                break;
            default:
                throw std::runtime_error("Invalid sbas enabled argument in mtk_3339::sbas_enabled_to_nmea()"); 
        }
        return generate_full_cmd(cmd);
    }

    std::string sbas_mode_to_nmea(sbas_mode_t sbas_mode)
    {
        std::string cmd;
        switch(sbas_mode)
        {
            case SBAS_MODE_TESTING:
                cmd = "PMTK319,0";
                break;
            case SBAS_MODE_INTEGRITY:
                cmd = "PMTK319,1";
                break;
            default:
                throw std::runtime_error("Invalid sbas mode argument in mtk_3339::sbas_mode_to_nmea()"); 
        }
        return generate_full_cmd(cmd);
    }

    settings_t::settings_t(std::string device,
                           baud_t baud_rate,
                           update_rate_t update_ms,
                           gps_output_t gps_out,
                           antenna_output_t antenna_output,
                           sbas_enabled_t sbas_enabled,
                           sbas_mode_t sbas_mode) : device(device),
                                                    baud_rate(baud_rate),
                                                    update_ms(update_ms),
                                                    gps_out(gps_out),
                                                    antenna_output(antenna_output),
                                                    sbas_enabled(sbas_enabled),
                                                    sbas_mode(sbas_mode) {};

    settings_t::settings_t(YAML::Node config_file_node)
    {
        for(YAML::const_iterator it = config_file_node.begin(); it != config_file_node.end(); ++it)
        {
            if(it->first.as<std::string>() == "device")
            {
                this->device = it->second.as<std::string>();
                continue;
            }

            if(it->first.as<std::string>() == "baud_rate")
            {
                this->baud_rate = int_to_baud(it->second.as<int>());
                continue;
            }

            if(it->first.as<std::string>() == "update_ms")
            {
                this->update_ms = int_to_update_rate(it->second.as<int>());
                continue;
            }

            if(it->first.as<std::string>() == "gps_out")
            {
                for(YAML::const_iterator gps_out_it = it->second.begin(); gps_out_it != it->second.end(); ++gps_out_it)
                {
                    if(gps_out_it->first.as<std::string>() == "gll")
                    {
                        this->gps_out.gll = gps_out_it->second.as<unsigned int>();
                    }
                    if(gps_out_it->first.as<std::string>() == "rmc")
                    {
                        this->gps_out.rmc = gps_out_it->second.as<unsigned int>();
                    }
                    if(gps_out_it->first.as<std::string>() == "vtg")
                    {
                        this->gps_out.vtg = gps_out_it->second.as<unsigned int>();
                    }
                    if(gps_out_it->first.as<std::string>() == "gga")
                    {
                        this->gps_out.gga = gps_out_it->second.as<unsigned int>();
                    }
                    if(gps_out_it->first.as<std::string>() == "gsa")
                    {
                        this->gps_out.gsa = gps_out_it->second.as<unsigned int>();
                    }
                    if(gps_out_it->first.as<std::string>() == "gsv")
                    {
                        this->gps_out.gsv = gps_out_it->second.as<unsigned int>();
                    }
                    if(gps_out_it->first.as<std::string>() == "antenna_status")
                    {
                        this->antenna_output = (gps_out_it->second.as<unsigned int>() != 0) ?
                                                SEND_ANTENNA :
                                                NO_SEND_ANTENNA;
                    }
                }
            }

            if(it->first.as<std::string>() == "sbas")
            {
                for(YAML::const_iterator sbas_it = it->second.begin(); sbas_it != it->second.end(); ++sbas_it)
                {
                    if(sbas_it->first.as<std::string>() == "enabled")
                    {
                        this->sbas_enabled = (sbas_it->second.as<unsigned int>() == 0) ? 
                                              SBAS_DISABLED : 
                                              SBAS_ENABLED;
                    }
                    if(sbas_it->first.as<std::string>() == "mode")
                    {
                        this->sbas_mode = (sbas_it->second.as<unsigned int>() == 0) ? 
                                           SBAS_MODE_TESTING : 
                                           SBAS_MODE_INTEGRITY;
                    }
                }
            }
        }
    }

    std::string generate_full_cmd(std::string cmd)
    {
        std::string checksum = generate_checksum(cmd);
        return cmd = "$" + cmd + "*" + checksum;
    }

    std::string generate_checksum(std::string cmd)
    {
        int checksum = 0;
        for(int i = 0; i < cmd.size(); i++)
        {
            checksum ^= cmd[i];
        }
        std::stringstream ret_ss;
        ret_ss << std::hex << checksum;
        return ret_ss.str();
    }

}
