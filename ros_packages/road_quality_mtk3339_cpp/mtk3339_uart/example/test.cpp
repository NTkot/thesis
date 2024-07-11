#include <iostream>
#include <chrono>

#include "mtk_serial.hpp"
#include "mtk_utils.hpp"

int main()
{
    mtk_3339::settings_t settings;
    
    settings.baud_rate = mtk_3339::BAUD_57600;
    settings.gps_out.gll = 1;
    settings.gps_out.rmc = 1;
    settings.gps_out.vtg = 1;
    settings.gps_out.gga = 1;
    settings.gps_out.gsa = 1;
    settings.gps_out.gsv = 1;
    settings.antenna_output = mtk_3339::SEND_ANTENNA;

    mtk_3339::MTK3339Serial obj(settings);
    
    obj.init();

    while(true)
    {
        // auto start = std::chrono::high_resolution_clock::now();
        obj.read_char();
        // auto stop = std::chrono::high_resolution_clock::now();
        // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        // std::cout << "Time needed for read_char(): " << duration.count() << "us" << std::endl;

        int ret = obj.parse_nmea();
        if(ret == 1)
        {
            std::cout << obj.gpgll_to_str() << std::endl;
        }
        else if(ret == 2)
        {
            std::cout << obj.gprmc_to_str() << std::endl;
        }
        else if(ret == 3)
        {
            std::cout << obj.gpvtg_to_str()  << std::endl;
        }
        else if(ret == 4)
        {
            std::cout << obj.gpgga_to_str()  << std::endl;
        }
        else if(ret == 5)
        {
            std::cout << obj.gpgsa_to_str()  << std::endl;
        }
        else if(ret == 6)
        {
            std::cout << obj.gpgsv_to_str()  << std::endl;
        }
        else if(ret == 7)
        {
            std::cout << obj.pcd11_to_str() << std::endl;
        }
    }
}