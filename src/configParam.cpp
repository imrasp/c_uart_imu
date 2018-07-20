#include "configParam.h"

int ConfigParam::cameraid;
int ConfigParam::baudrate;
int ConfigParam::fps;
int ConfigParam::sec;
int ConfigParam::gpstime;
int ConfigParam::slam_position_update;
int ConfigParam::gps_position_update;

string ConfigParam::uart_name;
string ConfigParam::setting;
string ConfigParam::record_path;

ConfigParam::ConfigParam()
{
}

ConfigParam::ConfigParam(int argc, char **argv)
{
    parse_commandline(argc, argv);
    readParams();
    // setting = (char *) "../config/config.yaml";
}

ConfigParam::~ConfigParam()
{
}

void ConfigParam::readParams(){
    cv::FileStorage fSettings(setting, cv::FileStorage::READ);
    baudrate = fSettings["system.baudrate"];
    fSettings["system.uart_name"] >> uart_name;
    fSettings["system.record_path"] >> record_path;
    sec = fSettings["system.sec"];
    cameraid = fSettings["system.cameraid"];
    fps = fSettings["system.fps"];
    gpstime = fSettings["system.gpstime"];
    slam_position_update = fSettings["system.slam_position_update"];
    gps_position_update = fSettings["system.gps_position_update"];

    cout << "\n-------------------------------------" << endl;
    cout << "Parameters :" << endl;
    cout << "UART name " << uart_name << " baudrate " << baudrate << endl;
    cout << "Save result at " << record_path << endl;
    cout << "FPS : " << fps << endl;
    cout << "Camera : /dev/video" << cameraid << endl;
    cout << "Record time (sec.) : " << sec << endl;
    cout << "-------------------------------------\n" << endl;
}

//   Parse Command Line
void ConfigParam::parse_commandline(int argc, char **argv) {
    // string for command line usage
    const char *commandline_usage = "usage: -s <path_to_setting>";

    // Read input arguments
    for (int i = 1; i < argc; i++) { // argv[0] is "mavlink"

        // Help
        if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            printf("%s\n", commandline_usage);
            throw EXIT_FAILURE;
        }
        // Path to setting
        if (strcmp(argv[i], "-s") == 0 || strcmp(argv[i], "--setting") == 0) {
            if (argc > i + 1) {
                setting = argv[i + 1];

            } else {
                printf("%s\n", commandline_usage);
                throw EXIT_FAILURE;
            }
        }
    }
    // end: for each input argument

    return;
}
