//
// Created by rasp on 25/6/2561.
//

#include "location_manager.h"

Location_Manager::Location_Manager(bool _update_gps_position, bool _update_slam_position, IMU_Recorder *imu_recorder_)
        : initializeGeodetic(false),
          update_gps_position(_update_gps_position), update_slam_position(_update_slam_position),
          b_pixhawk_time_ref(false), imu_recorder(imu_recorder_) {

    geodeticConverter = new geodetic_converter::GeodeticConverter();
    cx = 0;
    cy = 0;
    cz = 0;
    c_local_timestamp = 0;
    c_lat = 0;
    c_lon = 0;
    c_alt = 0;
    c_global_timestamp = 0;

    mutex_localpose = PTHREAD_MUTEX_INITIALIZER;
    mutex_globalpose = PTHREAD_MUTEX_INITIALIZER;

    time_to_exit = false;
    boost::thread threadInitialGeodetic = boost::thread(&Location_Manager::set_initial_geodetic_pose, this);
}

Location_Manager::~Location_Manager() {

}

void Location_Manager::set_initial_geodetic_pose() {
    std::cout << "Start set_initial_geodetic_pose thread..." << std::endl;
    while (!time_to_exit) {
        std::cout << "local time = " << c_local_timestamp << " global time = " << c_global_timestamp << std::endl;
        if (c_local_timestamp < c_global_timestamp && c_local_timestamp > 0) {

            pthread_mutex_lock(&mutex_localpose);
            pthread_mutex_lock(&mutex_globalpose);

            geodeticConverter->initialiseReference(c_lat / 10e7, c_lon / 10e7, c_alt / 10e7);

            // get initial position use for interpolation
            uint64_t i_time = c_local_timestamp;
            double i_x = cx;
            double i_y = cy;
            double i_z = cz;
            std::cout << "1st NED position is " << i_x << ", " << i_y << ", " << i_z << std::endl;
            pthread_mutex_unlock(&mutex_localpose);
            // get second position for interpolation
            uint64_t f_time;
            double f_x, f_y, f_z;
            while (c_local_timestamp != i_time) {
                pthread_mutex_lock(&mutex_localpose);
                f_time = c_local_timestamp;
                f_x = cx;
                f_y = cy;
                f_z = cz;
                pthread_mutex_unlock(&mutex_localpose);
            }
            std::cout << "2st NED position is " << f_x << ", " << f_y << ", " << f_z << std::endl;

            // perform interpolation
            init_nedx = interpolate(i_time, f_time, c_global_timestamp, i_x, f_x);
            init_nedy = interpolate(i_time, f_time, c_global_timestamp, i_y, f_y);
            init_nedz = interpolate(i_time, f_time, c_global_timestamp, i_z, f_z);

            initializeGeodetic = true;

            std::cout << "\n\n initialize geodetic position complete with \n"
                      << "timestamp : " << c_global_timestamp << std::endl
                      << "GPS position : " << c_lat << ", " << c_lon << ", " << c_alt << std::endl
                      << "NED position : " << init_nedx << ", " << init_nedy << ", " << init_nedz
                      << "\n ------------------------------------------------ \n\n";

            pthread_mutex_unlock(&mutex_globalpose);
            break;
        }
    }
}

void Location_Manager::set_local_position(uint32_t timestamp, double x, double y, double z) {
    std::cout << "set local pose \n";
    pthread_mutex_lock(&mutex_localpose);
    c_local_timestamp = timestamp;
    cx = x;
    cy = y;
    cz = z;
    pthread_mutex_unlock(&mutex_localpose);
}

void Location_Manager::set_global_position(uint32_t timestamp, double lat, double lon, double alt) {
    std::cout << "set global pose \n";
    pthread_mutex_lock(&mutex_globalpose);
    c_global_timestamp = timestamp;
    c_lat = lat;
    c_lon = lon;
    c_alt = alt;
    pthread_mutex_unlock(&mutex_globalpose);
}

void Location_Manager::set_scaled_imu(uint32_t boot_timestamp, int16_t xacc, int16_t yacc, int16_t zacc, int16_t xgyro,
                                      int16_t ygyro, int16_t zgyro) {
    /**
      * uint32_t time_boot_ms; ///< Timestamp (milliseconds since system boot)
      * int16_t xacc; ///< X acceleration (mg)
      * int16_t yacc; ///< Y acceleration (mg)
      * int16_t zacc; ///< Z acceleration (mg)
      * int16_t xgyro; ///< Angular speed around X axis (millirad /sec)
      * int16_t ygyro; ///< Angular speed around Y axis (millirad /sec)
      * int16_t zgyro; ///< Angular speed around Z axis (millirad /sec)
      *
      * convert acceleration to m/s^2 via
      * 1 acceleration of gravity (g)  =  9.80665 meter/square second (m/s^2)
      * 1 milli acceleration of gravity (mg) = 0.00980665 meter/square second (m/s^2)
      *
      * convert millirad /sec to rad/sec via
      * 1 millirad /sec = 1/1000 rad/sec
     **/

    uint64_t c_timestamp;
    // convert timestamp to unix time
    if (b_pixhawk_time_ref) c_timestamp = get_unixtime(boot_timestamp * 1e3);
    else c_timestamp = b_pixhawk_time_ref;

    double c_xacc = xacc * 9.80665 * 1e-3;
    double c_yacc = yacc * 9.80665 * 1e-3;
    double c_zacc = zacc * 9.80665 * 1e-3;

    double c_xgyro = xgyro * 1e-3;
    double c_ygyro = ygyro * 1e-3;
    double c_zgyro = zgyro * 1e-3;

    imu_recorder->
            add_imu_to_queue(c_timestamp, c_xacc, c_yacc, c_zacc, c_xgyro, c_ygyro, c_zgyro
    );

}

void Location_Manager::set_time(uint32_t boot_timestamp, uint64_t unix_timestamp) {
    pixhawk_ms_ref = boot_timestamp;
    pixhawk_unix_ref = unix_timestamp;
    b_pixhawk_time_ref = true;
}

// get microsecond time
uint64_t Location_Manager::get_unixtime(uint32_t time) {
    if (b_pixhawk_time_ref) {
        uint64_t timestamp_ms = pixhawk_unix_ref + (time - (pixhawk_ms_ref * 1000));
        return timestamp_ms * 1000;
    }
    return 0;
}

void Location_Manager::stream_global_position(uint32_t timestamp, double lat, double lon, double alt) {
//    nlohmann::json drone_position = {{"misison_id", 1}, {"time", timestamp}, {"lat", lat},{"lon", lon}, {"alt", alt}};
//    auto r = cpr::Post(cpr::Url{"192.168.1.132:3000/current_pos"},
//                       cpr::Body{drone_position.dump()}
//    );
}

double Location_Manager::interpolate(uint32_t x1, uint32_t x2, uint32_t x_predict, double y1, double y2) {
    // predicted_y =
    return y2 + (x_predict - x2) * ((y2 - y1) / (x2 - x1));
}

bool Location_Manager::isGeodeticInitialize() {
    return initializeGeodetic;
}

void Location_Manager::get_NED_from_geodetic(double lat, double lon, double alt, float *x, float *y, float *z) {
    double xx, yy, zz;
    geodeticConverter->geodetic2Ned(lat, lon, alt, &xx, &yy, &zz);
    *x = xx + init_nedx;
    *y = yy + init_nedy;
    *z = zz + init_nedz;
}

double Location_Manager::distanceInKmBetweenEarthCoordinates(double lat1, double lon1, double lat2, double lon2) {
    double earthRadiusKm = 6371;

    double dLat = geodeticConverter->deg2Rad(lat2 - lat1);
    double dLon = geodeticConverter->deg2Rad(lon2 - lon1);

    lat1 = geodeticConverter->deg2Rad(lat1);
    lat2 = geodeticConverter->deg2Rad(lat2);

    double a = sin(dLat / 2) * sin(dLat / 2) +
               sin(dLon / 2) * sin(dLon / 2) * cos(lat1) * cos(lat2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return earthRadiusKm * c;
}