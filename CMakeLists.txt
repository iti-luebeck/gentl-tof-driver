cmake_minimum_required(VERSION 2.8.3)
project(gentl_tof_driver)

IF(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake/Modules)
set(CMAKE_VERBOSE_MAKEFILE "ON")

find_package(GenTL REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++0x -march=native -Wl,--enable-new-dtags -Wl,-rpath,${GENTL_LIBRARY_DIR}")

set(THIS_PACKAGE_ROS_DEPS roscpp rospy std_msgs sensor_msgs tf cv_bridge pcl_ros)

find_package(catkin REQUIRED COMPONENTS ${THIS_PACKAGE_ROS_DEPS})

catkin_package(CATKIN_DEPENDS ${THIS_PACKAGE_ROS_DEPS})

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${GENTL_INCLUDE_DIRS}
)


add_executable(gentl_wrapper_node
    src/gentl_wrapper_node.cpp
    src/GtlSystemModule.cpp
    src/BaslerCameraInfo.cpp
)

target_link_libraries(gentl_wrapper_node ${catkin_LIBRARIES} ${GENTL_LIBRARIES})
