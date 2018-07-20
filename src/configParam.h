#ifndef ConfigParam_H

#define ConfigParam_H

#include <iostream>
#include <cstring>
#include <string>
#include <fstream>
#include <cstdlib>
#include <opencv2/opencv.hpp>

using namespace std;

class ConfigParam {
public:

    static string setting;

    static int cameraid;
    static int fps;
    static int baudrate;
    static int sec;
    static int gpstime;
    static int slam_position_update;
    static int gps_position_update;

    static string uart_name;
    static string mission_route;
    static string record_path;

    ConfigParam();
    ConfigParam(int argc, char **argv);
    ~ConfigParam();
    void initialization();
    void readParams();
    void parse_commandline(int argc, char **argv);

private:
    void fps2Timespace(int fps);
};


#endif
