cmake_minimum_required(VERSION 3.1)

set(PROJECT_NAME ComputerVison_tutorial)
project(${PROJECT_NAME})

#tutorial_1
add_executable(tutorial_1 tutorial_1/main.cpp CameraCalibration.cpp CameraCalibration.hpp loadimage.cpp loadimage.hpp)
find_package(OpenCV REQUIRED)
target_link_libraries(tutorial_1 PUBLIC ${OpenCV_LIBS})
target_include_directories(tutorial_1 PUBLIC ${OpenCV_INCLUDE_DIRS})

#tutorial_2
add_executable(tutorial_2 tutorial_2/main.cpp CameraCalibration.cpp CameraCalibration.hpp loadimage.cpp loadimage.hpp)
find_package(OpenCV REQUIRED)
target_link_libraries(tutorial_2 PUBLIC ${OpenCV_LIBS})
target_include_directories(tutorial_2 PUBLIC ${OpenCV_INCLUDE_DIRS})




