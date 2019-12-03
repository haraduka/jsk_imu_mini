cmake_minimum_required(VERSION 2.8.3)
project(jsk_imu_mini_msgs)

set(MSG_DEPS
  std_msgs
  geometry_msgs)

find_package(catkin REQUIRED COMPONENTS genmsg message_generation ${MSG_DEPS})

set(MSG_FILES
  Imu.msg
  DesireCoord.msg
  Potentio.msg
  Debug.msg
)

set(SRV_FILES
  ImuConfig.srv
)

add_message_files(DIRECTORY msg FILES ${MSG_FILES})

add_service_files(DIRECTORY srv FILES ${SRV_FILES})

generate_messages(DEPENDENCIES ${MSG_DEPS})

catkin_package(CATKIN_DEPENDS ${MSG_DEPS} message_runtime)

