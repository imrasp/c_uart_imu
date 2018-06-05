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
    bool bIMU = false;
    bool bSLAM = false;

    try {

        Mavlink_Control* mavlinkControl;
        IMU_Recorder* imu_recorder;
        Camera_Recorder* cameraRecorder;

        std::cout << "\nConfigurating params \n";
        ConfigParam configParam(argc, argv);
        std::cout << "\nCreating log folder \n";
        Log log(configParam.record_path);

        if(bIMU)  imu_recorder = new IMU_Recorder(&configParam, true);
        if(bIMU) mavlinkControl = new Mavlink_Control(&configParam, imu_recorder);
        if(bCamera) cameraRecorder = new Camera_Recorder(&configParam, true);
        if(bIMU) mavlinkControl->start();
        if(bCamera) cameraRecorder->start();
        if(bIMU) mavlinkControl->cmd();
        else sleep(configParam.sec);
        if(bCamera) cameraRecorder->stop();
       
        if(bIMU){
imu_recorder->stop();
mavlinkControl->stop();
        }

        return 0;
    }
    catch (int error) {
        fprintf(stderr, "threw exception %i \n", error);
        return error;
    }
}
