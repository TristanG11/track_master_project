#ifndef NMEA_PARSER_HPP
#define NMEA_PARSER_HPP

#include <string>
#include <vector>
#include <map>
#include <memory>
class NmeaParser {
public:
    NmeaParser();
    ~NmeaParser();

    void parse_stream(const std::string &nmea_stream);
    void parse_gga(const std::string &nmea_stream);
    void parse_vtg(const std::string &nmea_stream);

    static std::vector<std::string> split(const std::string &stream, char delimiter);

    double get_lat(const std::string &latitude);
    double get_lon(const std::string &longitude);
    int get_service_flags(const std::string &constellation);
    int get_status_flags(const std::string &fix_quality);

    std::shared_ptr<std::map<std::string, std::string>> gga_msg;  
    std::shared_ptr<std::map<std::string, std::string>> vtg_msg;
};

#endif // NMEA_PARSER_HPP
