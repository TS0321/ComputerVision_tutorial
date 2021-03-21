cmake_minimum_required(VERSION 3.1)

set(PROJECT_NAME Calibration)
project(${PROJECT_NAME})
add_executable(${PROJECT_NAME} main.cpp ../../loadimage.hpp ../../loadimage.cpp ../../CameraCalibration.hpp ../../CameraCalibration.cpp)
find_package(OpenCV REQUIRED)
target_link_libraries(${PROJECT_NAME} PUBLIC ${OpenCV_LIBS})
target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS})





