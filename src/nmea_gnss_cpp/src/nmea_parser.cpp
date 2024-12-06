#include "nmea_gnss_cpp/nmea_parser.hpp"
#include <stdexcept>
#include <sstream>
#include <iostream>

const std::map<std::string, int> constellation_map = {
        {"GP", 1},  // GPS
        {"GL", 2},  // GLONASS
        {"GA", 4},  // Galileo
        {"BD", 8},  // BeiDou
        {"QZ", 16}  // QZSS
};

const std::map<std::string, int> service_map{
    {"0",-1},
    {"1",0},
    {"2",1},
    {"4",1},
    {"5",1},
};

NmeaParser::NmeaParser(){
    gga_msg =  std::make_shared<std::map<std::string, std::string>>();
    
    (*gga_msg)["time_utc"] = "";       // Exemple d'heure UTC
    (*gga_msg)["latitude"] = "";        // Exemple de latitude
    (*gga_msg)["longitude"] = "";        // Exemple de longitude
    (*gga_msg)["fix_quality"] = "";           // Exemple de qualité du fix GNSS
    (*gga_msg)["service"] = "";             // Type de service (GPS/GLONASS/etc.)
    (*gga_msg)["num_satellites"] = "";        // Nombre de satellites utilisés
    (*gga_msg)["altitude"] = "";           // Altitude en mètres

    vtg_msg = std::make_shared<std::map<std::string, std::string>>();

    (*vtg_msg)["time_utc"] = ""; 
    (*vtg_msg)["magnetic_course"]="";
    (*vtg_msg)["speed_knots"]= "";
    (*vtg_msg)["speed_kmh"]="";
}

NmeaParser::~NmeaParser(){}

void NmeaParser::parse_stream(const std::string &nmea_stream){
    
}
void NmeaParser::parse_gga(const std::string &nmea_stream){
    std::vector<std::string> tokens = NmeaParser::split(nmea_stream,',');

    if (tokens.size() < 15){
        std::cout << tokens.size();
        throw std::runtime_error("Invalid GGA sentence: not enough fields");
    }else{
        (*gga_msg)["time_utc"] = tokens[1];       
        (*gga_msg)["latitude"] = tokens[2] + ',' + tokens[3];        
        (*gga_msg)["longitude"] = tokens[4] + ',' + tokens[5];     
        (*gga_msg)["fix_quality"] = tokens[6];       
        (*gga_msg)["service"] = tokens[0];             
        (*gga_msg)["num_satellites"] = tokens[7];        
        (*gga_msg)["altitude"] = tokens[9];  
    }
}
void NmeaParser::parse_vtg(const std::string &nmea_stream){
    std::vector<std::string> tokens = NmeaParser::split(nmea_stream,',');

    if (tokens.size()< 8){
        throw std::runtime_error("Invalid VTG sentence: not enough fields");
    }else{
        (*vtg_msg)["time_utc"] = tokens[1];
        (*vtg_msg)["magnetic_course"] = tokens[3];
        (*vtg_msg)["speed_knots"] = tokens[5];
        (*vtg_msg)["speed_kmh"] = tokens[7];
    }   
}
double NmeaParser::get_lat(const std::string &latitude){
    std::vector<std::string> tokens = NmeaParser::split(latitude,',');
    if (tokens.size()!=2){
        throw std::runtime_error("Invalid token for latitude");
    }else 
    {
        std::string direction = tokens[1];

        std::string lat_str = tokens[0];
        int degrees = std::stoi(lat_str.substr(0,2));
        double minutes = std::stod(lat_str.substr(2));
        double decimal = degrees + (minutes / 60.0);

        return (direction == "N")?  decimal : -decimal;
    }
}

double NmeaParser::get_lon(const std::string &longitude){
    std::vector<std::string> tokens = NmeaParser::split(longitude,',');
    if (tokens.size()!=2){
        throw std::runtime_error("Invalid token for longitude");
    }else 
    {
        std::string direction = tokens[1];

        std::string lat_str = tokens[0];
        int degrees = std::stoi(lat_str.substr(0,3));
        double minutes = std::stod(lat_str.substr(3));
        double decimal = degrees + (minutes / 60.0);

        return (direction == "E")?  decimal : -decimal;
    }

}
int NmeaParser::get_service_flags(const std::string &constellation){
    auto it = constellation_map.find(constellation);
    if (it !=constellation_map.end()){
        return it->second;
    }
    throw std::runtime_error("Unknown constellation type: " + constellation);    
}

int NmeaParser::get_status_flags(const std::string &fix_quality){
    int fix = std::stoi(fix_quality);
    auto it = service_map.find(fix_quality);
    if(it!=service_map.end()){
        return it->second;
    }else{
        return -1;
    }
}

std::vector<std::string> NmeaParser::split(const std::string &stream, char delimiter){
    std::vector<std::string> tokens;
    std::stringstream ss(stream);
    std::string token;

    while(std::getline(ss,token,delimiter)){
        tokens.push_back(token);
    }

    return tokens;
}