#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "pi_gpio.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    std::string package_share_path = ament_index_cpp::get_package_share_directory("road_quality_gpio");
    rclcpp::spin(std::make_shared<PiGpio>(package_share_path + "/gpio.yaml"));   
    rclcpp::shutdown();
    return 0;
}