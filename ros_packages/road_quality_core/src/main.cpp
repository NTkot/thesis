#include "rclcpp/rclcpp.hpp"

#include "nucleus.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Nucleus>());
    rclcpp::shutdown();
    return 0;
}