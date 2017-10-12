cmake_minimum_required(VERSION 2.8.3)
project(drive_test)

add_definitions(-lwiringPi -lpthread)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package( OpenCV REQUIRED )
include_directories(include)

add_executable(drive_test src/drive_test.cpp include/MiniPID/MiniPID.cpp)
target_link_libraries(drive_test wiringPi ${OpenCV_LIBS})

add_executable(balance src/balance.cpp include/MiniPID/MiniPID.cpp include/RPIO/pwm.c include/RPIO/mailbox.c)
target_link_libraries(balance wiringPi ${OpenCV_LIBS})

#add_executable(old_drive_test src/old_drive_test.cpp include/MiniPID/MiniPID.cpp)
#target_link_libraries(old_drive_test wiringPi)

add_executable(imu_test src/imu_test.cpp)

#add_executable(OpenCV_Test src/OpenCV_Test.cpp)
#target_link_libraries(OpenCV_Test ${OpenCV_LIBS})

#add_executable(MiniIMU9AHRS src/MiniIMU9AHRS.cpp)
#target_link_libraries(MiniIMU9AHRS wiringPi LSM6-arduino LIS3MDL-arduino)


