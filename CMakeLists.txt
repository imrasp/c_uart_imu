cmake_minimum_required(VERSION 3.2)
project(c_uart_interface_example)

# default built type
IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF()

set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV REQUIRED)
#find_package(Eigen3 REQUIRED)
find_package(Qt5Gui)
find_package(Qt5Core)
find_package(Qt5Widgets)
find_package(Qt5Test)
find_package(Qt5Concurrent)
find_package(Qt5OpenGL)

set(SOURCE_FILES
        src/autopilot_interface.cpp
        src/autopilot_interface.h
        src/configParam.cpp
        src/configParam.h
        main.cpp
        src/mavlink_control.cpp
        src/mavlink_control.h
        src/serial_port.cpp
        src/serial_port.h
        src/camera_recorder.cpp
        src/camera_recorder.h
        src/log.cpp
        src/log.h
        src/imu_recorder.cpp
        src/imu_recorder.h
        src/time_recorder.cpp
        src/time_recorder.h
        src/location_manager.cpp)

include_directories(
        ${CMAKE_SOURCE_DIR}/mavlink_new/include/mavlink/v1.0
        ${EIGEN3_INCLUDE_DIR}
)

add_executable(c_uart_interface_example ${SOURCE_FILES})

target_link_libraries(
        c_uart_interface_example
        pthread

        boost_system
        boost_thread
        boost_filesystem
        boost_date_time
        boost_chrono

        opencv_core
        opencv_highgui
        opencv_calib3d

        ${EIGEN3_LIBS}
)
