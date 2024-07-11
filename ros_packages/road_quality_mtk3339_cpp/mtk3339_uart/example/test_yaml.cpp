#include <iostream>
#include "yaml-cpp/yaml.h"
#include "mtk_utils.hpp"

int main()
{
    std::string path_to_yaml = "test.yaml";
    YAML::Node config_file_node = YAML::LoadFile(path_to_yaml.c_str());

    mtk_3339::settings_t settings_obj(config_file_node);

    std::cout << "device: " << settings_obj.device << "\n" <<
                 "baud_rate: " << mtk_3339::baud_to_str(settings_obj.baud_rate) << "\n" <<
                 "update_ms: " << mtk_3339::update_rate_to_str(settings_obj.update_ms) << "\n" <<
                 "gps_out:\n  gll: " << settings_obj.gps_out.gll << "\n" <<
                           "  rmc: " << settings_obj.gps_out.rmc << "\n" <<
                           "  vtg: " << settings_obj.gps_out.vtg << "\n" <<
                           "  gga: " << settings_obj.gps_out.gga << "\n" <<
                           "  gsa: " << settings_obj.gps_out.gsa << "\n" <<
                           "  gsv: " << settings_obj.gps_out.gsv << "\n" <<
                           "  antenna_output: " << settings_obj.antenna_output << "\n" 
                 "sbas:\n  enabled: " << settings_obj.sbas_enabled << "\n" <<
                        "  mode:    " << settings_obj.sbas_mode << "\n" << std::endl;

    return 0;
}