#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "mtk3339_reader.hpp"
#include "mtk_utils.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    std::string package_share_path = ament_index_cpp::get_package_share_directory("road_quality_mtk3339_cpp");
    YAML::Node config_file = YAML::LoadFile((package_share_path + "/gps.yaml").c_str());
    mtk_3339::settings_t usr_settings(config_file);

    rclcpp::spin(std::make_shared<MTK3339Reader>(usr_settings));
    rclcpp::shutdown();
    return 0;
}