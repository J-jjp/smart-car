cmake_minimum_required(VERSION 3.0.0)  
project(FPS_Test)  
  
find_package(OpenCV REQUIRED)  
  
include_directories(${OpenCV_INCLUDE_DIRS})  
  
add_executable(fps_test opencv.cpp)  
target_link_libraries(fps_test ${OpenCV_LIBS})
