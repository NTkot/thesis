#ifndef MTK_TYPES_HPP
#define MTK_TYPES_HPP

#include <string>

#include "yaml-cpp/yaml.h"


namespace mtk_3339
{
    /*** NMEA MESSAGES TYPEDEFS ***/
    /**
     * @enum nmea_pcd11_t
     * @brief Represents NMEA PCD11 message types in the GPS module.
     * 
     * This enumeration is used to define constants for different types of PCD11
     * messages that can be received from the MTK3339 GPS module. Each enumerator
     * corresponds to a specific PCD11 message type.
     */
    typedef enum nmea_pcd11_t
    {
        INTERNAL_ANTENNA_USED = 1,
        EXTERNAL_ANTENNA_USED,
        EXTERNAL_ANTENNA_SHORTED,
        ANTENNA_USED_ERROR
    } nmea_pcd11_t;



    /*** SETTINGS TYPEDEFS ***/
    /**
     * @enum mtk_baud_t
     * @brief Type representing the baud rates for the MTK3339 UART communication.
     */
    typedef enum baud_t
    {
        BAUD_4800,
        BAUD_9600,
        BAUD_14400,
        BAUD_19200,
        BAUD_38400,
        BAUD_57600,
        BAUD_115200,
    } baud_t;

    /**
     * @enum update_rate_t
     * @brief This enumeration defines the possible update rates that can be set
     *        for the MTK3339 GPS module, allowing for different frequencies 
     *        of position data updates based on user needs and application requirements.
     */
    typedef enum update_rate_t
    {
        UPDATE_RATE_100MS,
        UPDATE_RATE_200MS,
        UPDATE_RATE_500MS,
        UPDATE_RATE_1000MS,
    } update_rate_t;

    /**
     * @struct gps_output_t
     * @brief A structure representing the GPS output configurations. For each
     *        NMEA sentence, define the number of position fixes it takes to 
     *        output. A number of `0` corresponds to a disabled NMEA sentence.
     *        A positive number `n` means output sentence once every `n` position
     *        fixes. Usual value is 1 for GPRMC sentence, 0 for all others.
     */
    typedef struct gps_output_t
    {
        unsigned gll;
        unsigned rmc;
        unsigned vtg;
        unsigned gga;
        unsigned gsa;
        unsigned gsv;

        gps_output_t(unsigned gll = 0,
                     unsigned rmc = 1,
                     unsigned vtg = 0,
                     unsigned gga = 0,
                     unsigned gsa = 0,
                     unsigned gsv = 0);
    } gps_output_t;

    /**
     * @enum antenna_output_t
     * @brief Enumerates the antenna output options for the MTK3339 GPS module.
     */
    typedef enum antenna_output_t
    {
        SEND_ANTENNA,
        NO_SEND_ANTENNA
    } antenna_output_t;

    /**
     * @enum sbas_enabled_t
     * @brief Enumeration indicating whether SBAS (Satellite-Based Augmentation System) is enabled.
     */
    typedef enum sbas_enabled_t
    {
        SBAS_DISABLED,
        SBAS_ENABLED
    } sbas_enabled_t;

    /**
     * @enum sbas_mode_t
     * @brief Enumerates the SBAS (Satellite-Based Augmentation System) modes for testing and integrity.
     */
    typedef enum sbas_mode_t
    {
        SBAS_MODE_TESTING,
        SBAS_MODE_INTEGRITY
    } sbas_mode_t;

    /**
     * @struct settings_t
     * A structure to hold all the settings for the MTK3339 GPS module.
     */
    typedef struct settings_t
    {
        std::string device;
        baud_t baud_rate;
        update_rate_t update_ms;
        gps_output_t gps_out;
        antenna_output_t antenna_output;
        sbas_enabled_t sbas_enabled;
        sbas_mode_t sbas_mode;

        settings_t(std::string device = "/dev/ttyS0",
                   baud_t baud_rate = BAUD_9600,
                   update_rate_t update_ms = UPDATE_RATE_1000MS,
                   gps_output_t gps_out = gps_output_t(),
                   antenna_output_t antenna_output = SEND_ANTENNA,
                   sbas_enabled_t sbas_enabled = SBAS_ENABLED,
                   sbas_mode_t sbas_mode = SBAS_MODE_INTEGRITY);

        settings_t(YAML::Node config_file_node);
    } settings_t;




    /*** METHODS ***/
    /**
     * @brief Converts a baud rate represented by the baud_t enum to a string.
     * @param baud The baud rate as a baud_t enum.
     * @return A std::string representation of the baud rate.
     */
    std::string baud_to_str(baud_t baud);

    /**
     * @brief Converts a baud rate represented by the baud_t enum to an NMEA string command.
     * @param baud The baud rate as a baud_t enum.
     * @return A std::string containing the NMEA command for the baud rate.
     */
    std::string baud_to_nmea(baud_t baud);

    /**
     * @brief Converts an integer baud rate to the baud_t enum representation.
     * @param baud The baud rate as an integer.
     * @return The baud_t enum corresponding to the given baud rate.
     */
    baud_t int_to_baud(int baud);

    /**
     * @brief Converts an update rate represented by the update_rate_t enum to a string.
     * @param rate The update rate as an update_rate_t enum.
     * @return A std::string representation of the update rate.
     */
    std::string update_rate_to_str(update_rate_t rate);

    /**
     * @brief Converts an update rate represented by the update_rate_t enum to an NMEA string command.
     * @param rate The update rate as an update_rate_t enum.
     * @return A std::string containing the NMEA command for the update rate.
     */
    std::string update_rate_to_nmea(update_rate_t rate);

    /**
     * @brief Converts an integer update rate in milliseconds to the update_rate_t enum representation.
     * @param update_ms The update rate in milliseconds.
     * @return The update_rate_t enum corresponding to the given update rate.
     */
    update_rate_t int_to_update_rate(int update_ms);

    /**
     * @brief Converts GPS output mode to an NMEA string command.
     * @param gps_out The GPS output mode as a gps_output_t enum.
     * @return A std::string containing the NMEA command for the GPS output mode.
     */
    std::string gps_output_to_nmea(gps_output_t gps_out);

    /**
     * @brief Converts antenna output mode to an NMEA string command.
     * @param antenna_output The antenna output mode as an antenna_output_t enum.
     * @return A std::string containing the NMEA command for the antenna output mode.
     */
    std::string antenna_output_to_nmea(antenna_output_t antenna_output);

    /**
     * @brief Converts SBAS enabled status to an NMEA string command.
     * @param sbas_enabled The SBAS enabled status as an sbas_enabled_t enum.
     * @return A std::string containing the NMEA command for the SBAS enabled status.
     */
    std::string sbas_enabled_to_nmea(sbas_enabled_t sbas_enabled);

    /**
     * @brief Converts SBAS mode to an NMEA string command.
     * @param sbas_mode The SBAS mode as an sbas_mode_t enum.
     * @return A std::string containing the NMEA command for the SBAS mode.
     */
    std::string sbas_mode_to_nmea(sbas_mode_t sbas_mode);

    /**
     * @brief Returns full NMEA command
     * @param cmd NMEA command, without containing $, * and checksum field. Example: PMTK251,9600
     * @return String containing NMEA command (without \r\n at the end).
    */
    std::string generate_full_cmd(std::string cmd);

    /**
     * @brief Return checksum from NMEA sequence.
     * @param cmd NMEA sequence. Contents between $ and * characters. Example: PMTK251,9600
     * @return String containing checksum in hexadecimal format.
    */
    std::string generate_checksum(std::string cmd);
}

#endif
