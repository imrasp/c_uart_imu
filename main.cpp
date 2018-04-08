#include <iostream>

#include "src/mavlink_control.h"
#include "src/camera_recorder.h"
#include "src/configParam.h"
#include "src/log.h"
#include "src/imu_recorder.h"

using namespace std;

int main(int argc, char **argv);

int main(int argc, char **argv) {
    bool bCamera = true;
    bool bIMU = true;
    bool bSLAM = false;

    try {
        ConfigParam configParam(argc, argv);
        Log log(configParam.record_path);
        IMU_Recorder imu_recorder(true);
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
