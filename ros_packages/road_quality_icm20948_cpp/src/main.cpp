#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "icm20948_reader.hpp"
#include "icm20948_defs.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    std::string config_file = ament_index_cpp::get_package_share_directory("road_quality_icm20948_cpp") + "/imu.yaml";
    rclcpp::spin(std::make_shared<ICM20948Reader>(1, ICM20948_I2C_ADDR, config_file));
    rclcpp::shutdown();
    return 0;
}