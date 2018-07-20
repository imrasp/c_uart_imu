//
// Created by rasp on 29/4/2561.
//

#ifndef C_UART_IMU_LOCATION_MANAGER_H
#define C_UART_IMU_LOCATION_MANAGER_H

#include <stdio.h>
#include <iostream>
//#include <cpr/cpr.h>
//#include <nlohmann/json.hpp>
#include <boost/thread.hpp>

#include "geodetic_converter.cpp"
#include "imu_recorder.h"

class Location_Manager{
public:
    Location_Manager(bool _update_gps_position, bool _update_slam_position, IMU_Recorder* imu_recorder_);
    ~Location_Manager();

    bool initializeGeodetic;
    double init_nedx, init_nedy, init_nedz;
    void set_initial_geodetic_pose();
    void get_NED_from_geodetic(double lat, double lon, double alt, float *x, float *y, float *z);

    void set_local_position(uint32_t timestamp, double x, double y, double z);
    void set_global_position(uint32_t timestamp, double lat, double lon, double alt);
    void set_time(uint32_t boot_timestamp, uint64_t unix_timestamp);
    void set_scaled_imu(uint32_t boot_timestamp, int16_t xacc, int16_t yacc, int16_t zacc, int16_t xgyro, int16_t ygyro, int16_t zgyro );
    bool isGeodeticInitialize();

    double distanceInKmBetweenEarthCoordinates(double lat1, double lon1, double lat2, double lon2);

    double interpolate(uint32_t x1, uint32_t x2, uint32_t x_predict, double y1, double y2);
    void stream_global_position(uint32_t timestamp, double lat, double lon, double alt);

    uint64_t get_unixtime(uint32_t time);
private:
    geodetic_converter::GeodeticConverter *geodeticConverter;

    bool update_gps_position, update_slam_position;
    bool time_to_exit;
    double cx,cy,cz;
    double c_lat, c_lon, c_alt;
    uint32_t c_local_timestamp, c_global_timestamp;

    // time reference from pixhawk
    uint32_t pixhawk_ms_ref;
    uint64_t pixhawk_unix_ref;
    bool b_pixhawk_time_ref;

    IMU_Recorder* imu_recorder;

    pthread_mutex_t mutex_localpose, mutex_globalpose;
};

#endif //C_UART_IMU_LOCATION_MANAGER_H
