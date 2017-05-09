cmake_minimum_required(VERSION 2.8.3)
project(ec_drivers)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rtt_ros ec_hardware geometry_msgs
)
if(UNIX)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x")
endif()
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${USE_OROCOS_INCLUDE_DIRS})

orocos_service(ec_elmo_driver src/elmo_driver.cpp)
target_link_libraries(ec_elmo_driver ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})

orocos_service(ec_ati_driver src/ati_driver.cpp)
target_link_libraries(ec_ati_driver ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})

orocos_service(ec_imu_driver src/imu_driver.cpp)
target_link_libraries(ec_imu_driver ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})

orocos_generate_package(
  DEPENDS rtt_ros ec_hardware
)
