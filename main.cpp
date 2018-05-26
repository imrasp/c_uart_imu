#include <iostream>

#include "src/mavlink_control.h"
#include "src/camera_recorder.h"
#include "src/configParam.h"
#include "src/log.h"
#include "src/imu_recorder.h"

using namespace std;

int main(int argc, char **argv);

int main(int argc, char **argv) {
    bool bCamera = false;
    bool bIMU = true;
    bool bSLAM = false;

    try {
        std::cout << "\nConfigurating params \n";
        ConfigParam configParam(argc, argv);
        std::cout << "\nCreating log folder \n";
        Log log(configParam.record_path);
        std::cout << "\nStart IMU recorder thread \n";
        IMU_Recorder imu_recorder(&configParam, true);
        std::cout << "\nStart Mavlink_Control\n";
        Mavlink_Control mavlinkControl(&configParam, &imu_recorder);
        Camera_Recorder cameraRecorder(&configParam);
        if(bIMU){
            mavlinkControl.start();
        }
        if(bCamera) cameraRecorder.start();
        mavlinkControl.cmd();
        if(bCamera) cameraRecorder.stop();
       
        if(bIMU){
std::cout <<"mav con stopped \n";
imu_recorder.stop();
std::cout << "imu recorder stopped \n";
mavlinkControl.stop();}

        return 0;
    }
    catch (int error) {
        fprintf(stderr, "threw exception %i \n", error);
        return error;
    }
}
