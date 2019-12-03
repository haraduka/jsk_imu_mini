#include "ros/ros.h"
#include "jsk_imu_mini_msgs/Imu.h"

#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include <iostream>

ros::Publisher marker_pub;
visualization_msgs::Marker marker;

void printCallback( const jsk_imu_mini_msgs::Imu::ConstPtr& msg )
{
    double r,p,y;

    // convert quaternion formed orientation to rpy formed orientation
    tf::Quaternion quat( msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w );
    tf::Matrix3x3( quat ).getRPY( r, p, y );

    // print rpy
    std::cout << "roll  : " << r << std::endl;
    std::cout << "pitch : " << p << std::endl;
    std::cout << "yaw   : " << y << std::endl;
    std::cout << std::endl;

    // publish marker
    marker.pose.orientation.x = msg->orientation.x;
    marker.pose.orientation.y = msg->orientation.y;
    marker.pose.orientation.z = msg->orientation.z;
    marker.pose.orientation.w = msg->orientation.w;
    marker_pub.publish(marker);
}

int main(int argc, char **argv)
{
    if ( argc < 3 ) {
        std::cout << "usage: sample_rpy_publisher <topicname of jsk_imu_mini_msgs/Imu> <frame_id>" << std::endl;
        return 0;
    }
    std::string topicname_quaternion(argv[1]);
    std::string frame_id(argv[2]);

    ros::init( argc, argv, "rpy_publisher" );
    ros::NodeHandle nh;

    marker_pub = nh.advertise<visualization_msgs::Marker>("imu_marker",1);

    marker.header.frame_id = frame_id.c_str();
    marker.header.stamp = ros::Time::now();
    marker.ns = "";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.mesh_resource = "package://jsk_imu_mini_demo/meshes/marker.dae";
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;

    marker.lifetime = ros::Duration(0);

    ros::Subscriber sub = nh.subscribe( topicname_quaternion.c_str(), 10, printCallback );

    ros::spin();

    return 0;
}
