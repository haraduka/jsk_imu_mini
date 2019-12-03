#ifndef __ROS_NODE_H_
#define __ROS_NODE_H_

/* ros */
#include <ros.h>
/* inlcude ros message types*/
#include <std_msgs/Int8.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <jsk_imu_mini_msgs/Potentio.h>
/* inlcude ros related class */
//#include <examples/class.h>
#include <sensors/encoder/encoder.h>
#include <sensors/imu/imu_mpu9250.h>
#include <vector>
//define the duty
#define MAX_DUTY 54000 //forward
#define NEU_DUTY 40500 //neutral mid
#define MIN_DUTY 27000 //backward

class RosNode
{
public:
	RosNode(ros::NodeHandle* nh)
	{
		/* ros node initialize */
		nh_ = nh;

		/* ros publisher initialize */
		//radioflag_pub_ = new ros::Publisher("radioflag",&radioflag_msg);
		//odom_pub_ = new ros::Publisher("odometry/raw",&odom_msg);
		potentio_pub = new ros::Publisher("potentio/raw", &potentio_msg);
		//publish the msgs
		//nh_->advertise(*radioflag_pub_);
		//nh_->advertise(*odom_pub_);
		nh_->advertise(*potentio_pub);
		potentio_msg.potentio_length = 4;
		potentio_msg.potentio = (uint16_t*)malloc(sizeof(uint16_t)*4);

		/* ros subscriber initialize */
	}

	void publish(uint16_t arr[]){
		potentio_msg.stamp = nh_->now();
		for(int i=0; i<4; i++)
			potentio_msg.potentio[i] = arr[i];
		potentio_pub->publish(&potentio_msg);
	}

	~RosNode(){}

	/* ros node handle */
	ros::NodeHandle  *nh_;
	/* ros publisher */
	//ros::Publisher* radioflag_pub_; //radioflag_pub_
	//ros::Publisher* odom_pub_;  //odometry pub: twist message
	ros::Publisher* potentio_pub;
	/*  ros msgs */
	//std_msgs::Int8 radioflag_msg;
	//geometry_msgs::Twist odom_msg;
	jsk_imu_mini_msgs::Potentio potentio_msg;

private:
	/* aim linear and angular speed */
	//float aim_linear_spd = 0;
	//float aim_angular_spd = 0;
	//uint16_t LMotor_duty = NEU_DUTY;
	//uint16_t RMotor_duty = NEU_DUTY;
	//to avoid fall down, the acceleration and brake need to be very slow
	//reduce the Kp_l and encrease the Kd_l

	/* ros subscibers */

	/* ros call back function */

};
#endif

