//
// Created by rasp on 1/23/18.
// @djangogirl /Bangkok

#include "imu_recorder.h"
#include "location_manager.cpp"

IMU_Recorder::IMU_Recorder(ConfigParam *configParam_, bool active_) : active(active_), time_to_exit(false),
                                                                      configParam(configParam_) {
    unEmptyIMU = PTHREAD_COND_INITIALIZER;
    emptyIMU = PTHREAD_COND_INITIALIZER;
    mutexIMU = PTHREAD_MUTEX_INITIALIZER;
}

IMU_Recorder::~IMU_Recorder() {

}

void IMU_Recorder::stop() {
    //std::cout << "stop imu record thread \n";
    time_to_exit = true;
    threadRecordIMU.join();
    std::cout << "Finish recording IMU." << std::endl;
}


void IMU_Recorder::start() {

    std::cout << "Start IMU record thread..." << std::endl;
    threadRecordIMU = boost::thread(&IMU_Recorder::write_imu_from_queue, this);

}

void IMU_Recorder::add_imu_to_queue(uint64_t timestamp, double xacc, double yacc, double zacc,
                                    double xgyro, double ygyro, double zgyro) {
    std::cout << "add imu to queue\n";
    imu_data imu;
    imu.timestamp = timestamp;
    imu.xacc = xacc;
    imu.yacc = yacc;
    imu.zacc = zacc;
    imu.xgyro = xgyro;
    imu.ygyro = ygyro;
    imu.zgyro = zgyro;

    // add IMU to queue
    pthread_mutex_lock(&mutexIMU);
    queueIMU.push(imu);
    pthread_cond_signal(&unEmptyIMU);
    pthread_mutex_unlock(&mutexIMU);
}

void IMU_Recorder::write_imu_from_queue() {

    std::string sep = ",";
    ofstream datasetimu;

    std::cout << "creating files for recording messages with header\n";
    datasetimu.open("./record_data/imu0.csv");
    datasetimu << "timestamp" << sep << "omega_x" << sep << "omega_y" << sep << "omega_z" << sep << "alpha_x" << sep
               << "alpha_y" << sep << "alpha_z" << "\n";

    // waiting for imu message
//    pthread_mutex_lock(&mutexIMU);
//    pthread_cond_wait(&unEmptyIMU, &mutexIMU);
//    pthread_mutex_unlock(&mutexIMU);

    while (!time_to_exit) {// || !autopilot_interface->queueIMU.empty()) {
        std::cout << "write an imu\n";
        pthread_mutex_lock(&mutexIMU);
        if (queueIMU.empty())
            pthread_cond_wait(&unEmptyIMU, &mutexIMU);

        datasetimu << queueIMU.front().timestamp << sep
                   << queueIMU.front().xgyro << sep
                   << queueIMU.front().ygyro << sep
                   << queueIMU.front().zgyro << sep
                   << queueIMU.front().xacc << sep
                   << queueIMU.front().yacc << sep
                   << queueIMU.front().zacc << endl;

        queueIMU.pop();
    }
}