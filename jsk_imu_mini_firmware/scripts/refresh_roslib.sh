#!/bin/sh

DIRECTORY_MAIN=$(rospack find jsk_imu_mini_firmware)
DIRECTORY_ROS_LIB=$DIRECTORY_MAIN/ros_lib/
DIRECTORY_TEMP=$DIRECTORY_MAIN/temp/

# 一時ディレクトリの作成
if [ ! -e $DIRECTORY_TEMP ]; then
    mkdir $DIRECTORY_TEMP
fi

# msgファイルなどのbuild
catkin build jsk_imu_mini_msgs

# non-platformbasedなheader類の生成
rosrun rosserial_client make_libraries $DIRECTORY_TEMP

# 新しいファイル類の移動
cp -r $DIRECTORY_TEMP/ros_lib/jsk_imu_mini_msgs $DIRECTORY_ROS_LIB
cp -r $DIRECTORY_TEMP/ros_lib/rosserial_msgs $DIRECTORY_ROS_LIB
cp -r $DIRECTORY_TEMP/ros_lib/geometry_msgs $DIRECTORY_ROS_LIB
cp -r $DIRECTORY_TEMP/ros_lib/std_msgs $DIRECTORY_ROS_LIB

# 一時ディレクトリの削除
if [ -e $DIRECTORY_TEMP ]; then
    rm -rf $DIRECTORY_TEMP
fi
