#include <thread>
#include <stdexcept>

#include <nmea.h>
#include <nmea/gpgll.h>
#include <nmea/gprmc.h>
#include <nmea/gpvtg.h>
#include <nmea/gpgga.h>
#include <nmea/gpgsa.h>
#include <nmea/gpgsv.h>

#include "rclcpp/rclcpp.hpp"

#include "mtk_utils.hpp"

#include "mtk3339_reader.hpp"


MTK3339Reader::MTK3339Reader(mtk_3339::settings_t usr_settings) : Node("mtk3339_reader"),
                                                                  _lib_obj(usr_settings)
{
    this->_gps_init();

    this->_ros_init();

    this->_start_reader_thread();

    RCLCPP_INFO(this->get_logger(), "GPS driver initialized with the following options:\n"
                                    "Device: %s\n"
                                    "Baud rate: %s\n"
                                    "Update period: %sms\n"
                                    "Output messages:\n"
                                    "  GPGLL: %u\n"
                                    "  GPRMC: %u\n"
                                    "  GPVTG: %u\n"
                                    "  GPGGA: %u\n"
                                    "  GPGSA: %u\n"
                                    "  GPGSV: %u\n"
                                    "  PCD11: %d\n"
                                    "SBAS:\n"
                                    "  Enabled: %d\n"
                                    "  Mode:    %d", usr_settings.device.c_str(),
                                                     mtk_3339::baud_to_str(usr_settings.baud_rate).c_str(),
                                                     mtk_3339::update_rate_to_str(usr_settings.update_ms).c_str(),
                                                     usr_settings.gps_out.gll,
                                                     usr_settings.gps_out.rmc,
                                                     usr_settings.gps_out.vtg,
                                                     usr_settings.gps_out.gga,
                                                     usr_settings.gps_out.gsa,
                                                     usr_settings.gps_out.gsv,
                                                     (usr_settings.antenna_output == mtk_3339::SEND_ANTENNA) ? 1 : 0,
                                                     (usr_settings.sbas_enabled == mtk_3339::SBAS_ENABLED) ? 1 : 0,
                                                     (usr_settings.sbas_mode == mtk_3339::SBAS_MODE_INTEGRITY) ? 1 : 0);
}


void MTK3339Reader::_gps_init()
{
    if(this->_lib_obj.init() != 0)
    {
        throw(std::runtime_error("Could not initialize GPS sensor"));
    }
}


void MTK3339Reader::_ros_init()
{
    rclcpp::SensorDataQoS sensorQos;

    this->_gpgll_msg.header.frame_id = "gps";
    this->_gprmc_msg.header.frame_id = "gps";
    this->_gpvtg_msg.header.frame_id = "gps";
    this->_gpgga_msg.header.frame_id = "gps";
    this->_gpgsa_msg.header.frame_id = "gps";
    this->_gpgsv_msg.header.frame_id = "gps";
    this->_pcd11_msg.header.frame_id = "gps";

    if(this->_lib_obj.settings.gps_out.gll)
        this->_gpgll_publisher = this->create_publisher<road_quality_msgs::msg::Gpgll>("/gps/gpgll", sensorQos);
    if(this->_lib_obj.settings.gps_out.rmc)
        this->_gprmc_publisher = this->create_publisher<road_quality_msgs::msg::Gprmc>("/gps/gprmc", sensorQos);
    if(this->_lib_obj.settings.gps_out.vtg)
        this->_gpvtg_publisher = this->create_publisher<road_quality_msgs::msg::Gpvtg>("/gps/gpvtg", sensorQos);
    if(this->_lib_obj.settings.gps_out.gga)
        this->_gpgga_publisher = this->create_publisher<road_quality_msgs::msg::Gpgga>("/gps/gpgga", sensorQos);
    if(this->_lib_obj.settings.gps_out.gsa)
        this->_gpgsa_publisher = this->create_publisher<road_quality_msgs::msg::Gpgsa>("/gps/gpgsa", sensorQos);
    if(this->_lib_obj.settings.gps_out.gsv)
        this->_gpgsv_publisher = this->create_publisher<road_quality_msgs::msg::Gpgsv>("/gps/gpgsv", sensorQos);
    if(this->_lib_obj.settings.antenna_output == mtk_3339::SEND_ANTENNA)
        this->_pcd11_publisher = this->create_publisher<road_quality_msgs::msg::Pcd11>("/gps/pcd11", sensorQos);
}


void MTK3339Reader::_start_reader_thread()
{
    _reader_thread = std::thread(&MTK3339Reader::_reader_thread_work, this);
    _reader_thread.detach();
}


void MTK3339Reader::_reader_thread_work()
{
    int ret;
    
    while(rclcpp::ok())
    {
        this->_lib_obj.read_char();

        ret = this->_lib_obj.parse_nmea();
        if(ret == 1)
        {
            // GPGLL msg
            this->_gpgll_msg.header.stamp = rclcpp::Node::now();
            this->_cp_gpgll_struct_to_ros_msg();
            this->_gpgll_publisher->publish(this->_gpgll_msg);
        }
        else if(ret == 2)
        {
            // GPRMC msg
            this->_gprmc_msg.header.stamp = rclcpp::Node::now();
            this->_cp_gprmc_struct_to_ros_msg();
            this->_gprmc_publisher->publish(this->_gprmc_msg);
        }
        else if(ret == 3)
        {
            // GPVTG msg
            this->_gpvtg_msg.header.stamp = rclcpp::Node::now();
            this->_cp_gpvtg_struct_to_ros_msg();
            this->_gpvtg_publisher->publish(this->_gpvtg_msg);
        }
        else if(ret == 4)
        {
            // GPGGA msg
            this->_gpgga_msg.header.stamp = rclcpp::Node::now();
            this->_cp_gpgga_struct_to_ros_msg();
            this->_gpgga_publisher->publish(this->_gpgga_msg);
        }
        else if(ret == 5)
        {
            // GPGSA msg
            this->_gpgsa_msg.header.stamp = rclcpp::Node::now();
            this->_cp_gpgsa_struct_to_ros_msg();
            this->_gpgsa_publisher->publish(this->_gpgsa_msg);
        }
        else if(ret == 6)
        {
            // GPGSV msg
            this->_gpgsv_msg.header.stamp = rclcpp::Node::now();
            this->_cp_gpgsv_struct_to_ros_msg();
            this->_gpgsv_publisher->publish(this->_gpgsv_msg);
        }
        else if(ret == 7)
        {
            // PCD11 msg
            this->_pcd11_msg.header.stamp = rclcpp::Node::now();
            this->_pcd11_msg.antenna_status = this->_lib_obj.pcd11;
            this->_pcd11_publisher->publish(this->_pcd11_msg);
        }
    }
}


void MTK3339Reader::_cp_gpgll_struct_to_ros_msg()
{
    // Date does not exist in GPGLL message
    this->_gpgll_msg.gps_time.day           = 0;
    this->_gpgll_msg.gps_time.month         = 0;
    this->_gpgll_msg.gps_time.year          = 0;
    this->_gpgll_msg.gps_time.hours         = this->_lib_obj.gpgll.time.tm_hour;
    this->_gpgll_msg.gps_time.minutes       = this->_lib_obj.gpgll.time.tm_min;
    this->_gpgll_msg.gps_time.seconds       = this->_lib_obj.gpgll.time.tm_sec;
    this->_gpgll_msg.gps_time.milli_seconds = 0;

    this->_gpgll_msg.longitude_deg = (double)this->_lib_obj.gpgll.longitude.degrees + 
                                             this->_lib_obj.gpgll.longitude.minutes / 60.0;
    this->_gpgll_msg.longitude_dir = (this->_lib_obj.gpgll.longitude.cardinal == NMEA_CARDINAL_DIR_EAST) ?
                                      this->_gpgll_msg.LONGITUDE_EAST :
                                    ((this->_lib_obj.gpgll.longitude.cardinal == NMEA_CARDINAL_DIR_WEST) ?
                                      this->_gpgll_msg.LONGITUDE_WEST :
                                      NMEA_CARDINAL_DIR_UNKNOWN);
    this->_gpgll_msg.longitude_deg *= (this->_gpgll_msg.longitude_dir == this->_gpgll_msg.LONGITUDE_WEST) ? 
                                      -1.0 : 
                                       1.0;

    this->_gpgll_msg.latitude_deg = (double)this->_lib_obj.gpgll.latitude.degrees + 
                                            this->_lib_obj.gpgll.latitude.minutes / 60.0;
    this->_gpgll_msg.latitude_dir = (this->_lib_obj.gpgll.latitude.cardinal == NMEA_CARDINAL_DIR_NORTH) ?
                                     this->_gpgll_msg.LATITUDE_NORTH :
                                   ((this->_lib_obj.gpgll.latitude.cardinal == NMEA_CARDINAL_DIR_SOUTH) ?
                                     this->_gpgll_msg.LATITUDE_SOUTH :
                                     NMEA_CARDINAL_DIR_UNKNOWN);
    this->_gpgll_msg.latitude_deg *= (this->_gpgll_msg.latitude_dir == this->_gpgll_msg.LATITUDE_SOUTH) ?
                                     -1.0 : 
                                      1.0;
}


void MTK3339Reader::_cp_gprmc_struct_to_ros_msg()
{
    this->_gprmc_msg.gps_time.day           = this->_lib_obj.gprmc.date_time.tm_mday;
    this->_gprmc_msg.gps_time.month         = this->_lib_obj.gprmc.date_time.tm_mon + 1;
    this->_gprmc_msg.gps_time.year          = this->_lib_obj.gprmc.date_time.tm_year + 1900;
    this->_gprmc_msg.gps_time.hours         = this->_lib_obj.gprmc.date_time.tm_hour;
    this->_gprmc_msg.gps_time.minutes       = this->_lib_obj.gprmc.date_time.tm_min;
    this->_gprmc_msg.gps_time.seconds       = this->_lib_obj.gprmc.date_time.tm_sec;
    this->_gprmc_msg.gps_time.milli_seconds = 0;

    this->_gprmc_msg.longitude_deg = (double)this->_lib_obj.gprmc.longitude.degrees + 
                                             this->_lib_obj.gprmc.longitude.minutes / 60.0;
    this->_gprmc_msg.longitude_dir = (this->_lib_obj.gprmc.longitude.cardinal == NMEA_CARDINAL_DIR_EAST) ?
                                      this->_gprmc_msg.LONGITUDE_EAST :
                                    ((this->_lib_obj.gprmc.longitude.cardinal == NMEA_CARDINAL_DIR_WEST) ?
                                      this->_gprmc_msg.LONGITUDE_WEST :
                                      NMEA_CARDINAL_DIR_UNKNOWN);
    this->_gprmc_msg.longitude_deg *= (this->_gprmc_msg.longitude_dir == this->_gprmc_msg.LONGITUDE_WEST) ? 
                                      -1.0 : 
                                       1.0;

    this->_gprmc_msg.latitude_deg = (double)this->_lib_obj.gprmc.latitude.degrees + 
                                            this->_lib_obj.gprmc.latitude.minutes / 60.0;
    this->_gprmc_msg.latitude_dir = (this->_lib_obj.gprmc.latitude.cardinal == NMEA_CARDINAL_DIR_NORTH) ?
                                     this->_gprmc_msg.LATITUDE_NORTH :
                                   ((this->_lib_obj.gprmc.latitude.cardinal == NMEA_CARDINAL_DIR_SOUTH) ?
                                     this->_gprmc_msg.LATITUDE_SOUTH :
                                     NMEA_CARDINAL_DIR_UNKNOWN);
    this->_gprmc_msg.latitude_deg *= (this->_gprmc_msg.latitude_dir == this->_gprmc_msg.LATITUDE_SOUTH) ?
                                     -1.0 : 
                                      1.0;

    this->_gprmc_msg.ground_speed_kmh = this->_lib_obj.gprmc.gndspd_knots * 1.852;

    this->_gprmc_msg.track_deg = this->_lib_obj.gprmc.track_deg;

    this->_gprmc_msg.mag_var_deg = this->_lib_obj.gprmc.magvar_deg;
    this->_gprmc_msg.mag_var_dir = (this->_lib_obj.gprmc.magvar_cardinal == NMEA_CARDINAL_DIR_EAST) ?
                                    this->_gprmc_msg.MAG_VAR_EAST : 
                                  ((this->_lib_obj.gprmc.magvar_cardinal == NMEA_CARDINAL_DIR_WEST) ?
                                    this->_gprmc_msg.MAG_VAR_WEST :
                                    NMEA_CARDINAL_DIR_UNKNOWN);

    this->_gprmc_msg.pos_valid = this->_lib_obj.gprmc.valid;
}


void MTK3339Reader::_cp_gpvtg_struct_to_ros_msg()
{
    this->_gpvtg_msg.ground_speed_knots = this->_lib_obj.gpvtg.gndspd_knots;

    this->_gpvtg_msg.ground_speed_kmh = this->_lib_obj.gpvtg.gndspd_kmph;

    this->_gpvtg_msg.track_deg = this->_lib_obj.gpvtg.track_deg;
}


void MTK3339Reader::_cp_gpgga_struct_to_ros_msg()
{
    // Date does not exist in GPGGA message
    this->_gpgga_msg.gps_time.day           = 0;
    this->_gpgga_msg.gps_time.month         = 0;
    this->_gpgga_msg.gps_time.year          = 0;
    this->_gpgga_msg.gps_time.hours         = this->_lib_obj.gpgga.time.tm_hour;
    this->_gpgga_msg.gps_time.minutes       = this->_lib_obj.gpgga.time.tm_min;
    this->_gpgga_msg.gps_time.seconds       = this->_lib_obj.gpgga.time.tm_sec;
    this->_gpgga_msg.gps_time.milli_seconds = 0;

    this->_gpgga_msg.longitude_deg = (double)this->_lib_obj.gpgga.longitude.degrees + 
                                             this->_lib_obj.gpgga.longitude.minutes / 60.0;
    this->_gpgga_msg.longitude_dir = (this->_lib_obj.gpgga.longitude.cardinal == NMEA_CARDINAL_DIR_EAST) ?
                                      this->_gpgga_msg.LONGITUDE_EAST :
                                    ((this->_lib_obj.gpgga.longitude.cardinal == NMEA_CARDINAL_DIR_WEST) ?
                                      this->_gpgga_msg.LONGITUDE_WEST :
                                      NMEA_CARDINAL_DIR_UNKNOWN);
    this->_gpgga_msg.longitude_deg *= (this->_gpgga_msg.longitude_dir == this->_gpgga_msg.LONGITUDE_WEST) ? 
                                      -1.0 : 
                                       1.0;

    this->_gpgga_msg.latitude_deg = (double)this->_lib_obj.gpgga.latitude.degrees + 
                                            this->_lib_obj.gpgga.latitude.minutes / 60.0;
    this->_gpgga_msg.latitude_dir = (this->_lib_obj.gpgga.latitude.cardinal == NMEA_CARDINAL_DIR_NORTH) ?
                                     this->_gpgga_msg.LATITUDE_NORTH :
                                   ((this->_lib_obj.gpgga.latitude.cardinal == NMEA_CARDINAL_DIR_SOUTH) ?
                                     this->_gpgga_msg.LATITUDE_SOUTH :
                                     NMEA_CARDINAL_DIR_UNKNOWN);
    this->_gpgga_msg.latitude_deg *= (this->_gpgga_msg.latitude_dir == this->_gpgga_msg.LATITUDE_SOUTH) ?
                                     -1.0 : 
                                      1.0;

    this->_gpgga_msg.altitude = this->_lib_obj.gpgga.altitude;
    this->_gpgga_msg.altitude_unit = this->_lib_obj.gpgga.altitude_unit;

    this->_gpgga_msg.undulation = this->_lib_obj.gpgga.undulation;
    this->_gpgga_msg.undulation_unit = this->_lib_obj.gpgga.undulation_unit;

    this->_gpgga_msg.n_satellites = this->_lib_obj.gpgga.n_satellites;

    this->_gpgga_msg.position_fix_type = this->_lib_obj.gpgga.position_fix;
}


void MTK3339Reader::_cp_gpgsa_struct_to_ros_msg()
{
    this->_gpgsa_msg.mode = (this->_lib_obj.gpgsa.mode == 'M') ? 
                             this->_gpgsa_msg.MODE_MANUAL :
                           ((this->_lib_obj.gpgsa.mode == 'A') ?
                             this->_gpgsa_msg.MODE_AUTOMATIC :
                             this->_lib_obj.gpgsa.mode);

    this->_gpgsa_msg.fix_type = (this->_lib_obj.gpgsa.fixtype == 1) ? 
                                 this->_gpgsa_msg.FIX_TYPE_NOT_AVAILABLE :
                               ((this->_lib_obj.gpgsa.fixtype == 2) ?
                                 this->_gpgsa_msg.FIX_TYPE_2D :
                               ((this->_lib_obj.gpgsa.fixtype == 3) ?
                                 this->_gpgsa_msg.FIX_TYPE_3D :
                                 this->_gpgsa_msg.FIX_TYPE_UNDEFINED));
    
    this->_gpgsa_msg.sat_ids[0]  = this->_lib_obj.gpgsa.satID_00;
    this->_gpgsa_msg.sat_ids[1]  = this->_lib_obj.gpgsa.satID_01;
    this->_gpgsa_msg.sat_ids[2]  = this->_lib_obj.gpgsa.satID_02;
    this->_gpgsa_msg.sat_ids[3]  = this->_lib_obj.gpgsa.satID_03;
    this->_gpgsa_msg.sat_ids[4]  = this->_lib_obj.gpgsa.satID_04;
    this->_gpgsa_msg.sat_ids[5]  = this->_lib_obj.gpgsa.satID_05;
    this->_gpgsa_msg.sat_ids[6]  = this->_lib_obj.gpgsa.satID_06;
    this->_gpgsa_msg.sat_ids[7]  = this->_lib_obj.gpgsa.satID_07;
    this->_gpgsa_msg.sat_ids[8]  = this->_lib_obj.gpgsa.satID_08;
    this->_gpgsa_msg.sat_ids[9]  = this->_lib_obj.gpgsa.satID_09;
    this->_gpgsa_msg.sat_ids[10] = this->_lib_obj.gpgsa.satID_10;
    this->_gpgsa_msg.sat_ids[11] = this->_lib_obj.gpgsa.satID_11;

    this->_gpgsa_msg.pdop = this->_lib_obj.gpgsa.pdop;
    this->_gpgsa_msg.hdop = this->_lib_obj.gpgsa.hdop;
    this->_gpgsa_msg.vdop = this->_lib_obj.gpgsa.vdop;
}


void MTK3339Reader::_cp_gpgsv_struct_to_ros_msg()
{
    this->_gpgsv_msg.n_sentences = this->_lib_obj.gpgsv.sentences;

    this->_gpgsv_msg.current_sentence = this->_lib_obj.gpgsv.sentence_number;

    this->_gpgsv_msg.n_satellites_view = this->_lib_obj.gpgsv.satellites;

    for(int i = 0; i < 4; i++)
    {
        this->_gpgsv_msg.sat_prn[i]           = this->_lib_obj.gpgsv.sat[i].prn;
        this->_gpgsv_msg.sat_elevation_deg[i] = this->_lib_obj.gpgsv.sat[i].elevation;
        this->_gpgsv_msg.sat_azimuth_deg[i]   = this->_lib_obj.gpgsv.sat[i].azimuth;
        this->_gpgsv_msg.sat_snr[i]           = this->_lib_obj.gpgsv.sat[i].snr;
    }
}
