#include <gtest/gtest.h>
#include "nmea_gnss_cpp/nmea_parser.hpp"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

std::vector<std::string> read_sentences_from_file(const std::string &file_path){
    std::vector<std::string> sentences;
    std::ifstream file(file_path);
    if (!file.is_open()){
        throw std::runtime_error("Could not open file: " + file_path);
    }
    std::string line;
    while(std::getline(file,line)){
        if(line.empty()){
            continue;
        }
        sentences.push_back(line);
    }
    file.close();
    return sentences;
}

TEST(NmeaParserTest, ValidGGASentence) {
    // Arrange
    NmeaParser parser;
    std::string valid_gga_sentence = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47";

    // Act
    EXPECT_NO_THROW(parser.parse_gga(valid_gga_sentence));

    // Assert
    auto gga_msg = parser.gga_msg;
    EXPECT_EQ((*gga_msg)["time_utc"], "123519");
    EXPECT_EQ((*gga_msg)["latitude"], "4807.038,N");
    EXPECT_EQ((*gga_msg)["longitude"], "01131.000,E");
    EXPECT_EQ((*gga_msg)["fix_quality"], "1");
    EXPECT_EQ((*gga_msg)["service"], "$GPGGA");
    EXPECT_EQ((*gga_msg)["num_satellites"], "08");
    EXPECT_EQ((*gga_msg)["altitude"], "545.4");
}


TEST(NmeaParserTest, InvalidGGASentenceTooFewFields) {
    NmeaParser parser;
    std::string invalid_gga_sentence = "$GPGGA,123519";
    EXPECT_ANY_THROW(parser.parse_gga(invalid_gga_sentence),std::runtime_error);
}
