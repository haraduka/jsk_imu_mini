cmake_minimum_required(VERSION 2.8.3)
project(jsk_imu_mini_demo)

find_package(
    catkin REQUIRED COMPONENTS roscpp rospy jsk_imu_mini_msgs tf
)

catkin_package(
    CATKIN_DEPENDS roscpp rospy jsk_imu_mini_msgs tf
    DEPENDS eigen
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(
    sample_rpy_publisher
    src/sample_rpy_publisher.cpp
)
target_link_libraries(
    sample_rpy_publisher
    ${catkin_LIBRARIES}
)
