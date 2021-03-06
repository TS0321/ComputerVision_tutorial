cmake_minimum_required(VERSION 3.1)

set(PROJECT_NAME ComputerVison_tutorial)
project(${PROJECT_NAME})
add_executable(tutorial_1 main.cpp)
find_package(OpenCV REQUIRED)
target_link_libraries(tutorial_1 PUBLIC ${OpenCV_LIBS})
target_include_directories(tutorial_1 PUBLIC ${OpenCV_INCLUDE_DIRS})





