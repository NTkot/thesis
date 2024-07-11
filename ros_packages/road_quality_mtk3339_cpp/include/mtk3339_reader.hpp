#ifndef MTK3339_READER_HPP
#define MTK3339_READER_HPP

#include <thread>

#include <nmea/gpgll.h>
#include <nmea/gprmc.h>
#include <nmea/gpvtg.h>
#include <nmea/gpgga.h>
#include <nmea/gpgsa.h>
#include <nmea/gpgsv.h>

#include "rclcpp/rclcpp.hpp"

#include "mtk_serial.hpp"
#include "mtk_utils.hpp"

#include "builtin_interfaces/msg/time.hpp"
#include "road_quality_msgs/msg/gpgll.hpp"
#include "road_quality_msgs/msg/gprmc.hpp"
#include "road_quality_msgs/msg/gpvtg.hpp"
#include "road_quality_msgs/msg/gpgga.hpp"
#include "road_quality_msgs/msg/gpgsa.hpp"
#include "road_quality_msgs/msg/gpgsv.hpp"
#include "road_quality_msgs/msg/pcd11.hpp"

class MTK3339Reader : public rclcpp::Node
{
    private:
        mtk_3339::MTK3339Serial _lib_obj;
        std::thread _reader_thread;

        road_quality_msgs::msg::Gpgll _gpgll_msg;
        road_quality_msgs::msg::Gprmc _gprmc_msg;
        road_quality_msgs::msg::Gpvtg _gpvtg_msg;
        road_quality_msgs::msg::Gpgga _gpgga_msg;
        road_quality_msgs::msg::Gpgsa _gpgsa_msg;
        road_quality_msgs::msg::Gpgsv _gpgsv_msg;
        road_quality_msgs::msg::Pcd11 _pcd11_msg;

        rclcpp::Publisher<road_quality_msgs::msg::Gpgll>::SharedPtr _gpgll_publisher;
        rclcpp::Publisher<road_quality_msgs::msg::Gprmc>::SharedPtr _gprmc_publisher;
        rclcpp::Publisher<road_quality_msgs::msg::Gpvtg>::SharedPtr _gpvtg_publisher;
        rclcpp::Publisher<road_quality_msgs::msg::Gpgga>::SharedPtr _gpgga_publisher;
        rclcpp::Publisher<road_quality_msgs::msg::Gpgsa>::SharedPtr _gpgsa_publisher;
        rclcpp::Publisher<road_quality_msgs::msg::Gpgsv>::SharedPtr _gpgsv_publisher;
        rclcpp::Publisher<road_quality_msgs::msg::Pcd11>::SharedPtr _pcd11_publisher;

        void _gps_init();
        void _ros_init();
        void _start_reader_thread();
        void _reader_thread_work();

        void _cp_gpgll_struct_to_ros_msg();
        void _cp_gprmc_struct_to_ros_msg();
        void _cp_gpvtg_struct_to_ros_msg();
        void _cp_gpgga_struct_to_ros_msg();
        void _cp_gpgsa_struct_to_ros_msg();
        void _cp_gpgsv_struct_to_ros_msg();

    public:
        MTK3339Reader(mtk_3339::settings_t usr_settings = mtk_3339::settings_t());
};

#endif