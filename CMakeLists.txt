cmake_minimum_required(VERSION 3.1)

set(PROJECT_NAME ComputerVison_tutorial)
project(${PROJECT_NAME})


option(GLEW_STATIC "Option description" ON)


#tutorial_1
add_executable(tutorial_1 tutorial_1/main.cpp CameraCalibration.cpp CameraCalibration.hpp loadimage.cpp loadimage.hpp)
find_package(OpenCV REQUIRED)
target_link_libraries(tutorial_1 PUBLIC ${OpenCV_LIBS})
target_include_directories(tutorial_1 PUBLIC ${OpenCV_INCLUDE_DIRS})

#tutorial_2
add_executable(tutorial_2 tutorial_2/main.cpp CameraCalibration.cpp CameraCalibration.hpp loadimage.cpp loadimage.hpp CalcPose.cpp CalcPose.hpp)
find_package(OpenCV REQUIRED)
target_link_libraries(tutorial_2 PUBLIC ${OpenCV_LIBS})
target_include_directories(tutorial_2 PUBLIC ${OpenCV_INCLUDE_DIRS})
find_package(Eigen3 REQUIRED)
target_include_directories(tutorial_2 PUBLIC ${EIGEN3_INCLUDE_DIRS})

#tutorial3
set(GLFW_DIR "c:/lib/GLFW")
set(GLEW_DIR "c:/lib/GLEW")

link_directories(${GLFW_DIR}/lib)
link_directories(${GLEW_DIR}/lib)
add_executable(tutorial_3 tutorial_3/main.cpp loadimage.cpp loadimage.hpp shader.hpp shader.cpp window.hpp window.cpp shape.hpp shape.cpp object.hpp object.cpp)
find_package(OpenGL REQUIRED)
target_link_libraries(tutorial_3 PUBLIC opengl32.lib)
target_include_directories(tutorial_3 PUBLIC ${GLFW_DIR}/include)
target_link_libraries(tutorial_3 PUBLIC glfw3.lib)
include_directories(${GLEW_DIR}/include)
target_link_libraries(tutorial_3 PUBLIC libglew32d.lib glew32d.lib)
find_package(OpenCV REQUIRED)
target_link_libraries(tutorial_3 PUBLIC ${OpenCV_LIBS})
target_include_directories(tutorial_3 PUBLIC ${OpenCV_INCLUDE_DIRS})
find_package(Eigen3 REQUIRED)
target_include_directories(tutorial_3 PUBLIC ${EIGEN3_INCLUDE_DIRS})



