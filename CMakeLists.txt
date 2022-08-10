cmake_minimum_required(VERSION 3.0.2)
project(azrael_mobile_driver)

find_package(iir)


## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)
#add_compile_definitions(DEBUG_PID)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME} src/driver.cpp)

target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -mcpu=cortex-a72 -mtune=cortex-a72 -O3)


## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
    wiringPi
    pthread
    phidget22
    rt
    iir
)

