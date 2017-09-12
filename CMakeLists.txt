cmake_minimum_required(VERSION 2.8.3)
project(drive_test)

add_definitions(-lwiringPi -lpthread)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(include)

add_executable(drive_test src/drive_test.cpp include/MiniPID/MiniPID.cpp)

target_link_libraries(drive_test wiringPi)

