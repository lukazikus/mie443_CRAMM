#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include <eStop.h>

#include <stdio.h>
#include <cmath>

#include <cv.h>
#include <opencv2/highgui/highgui.hpp>
#include "highgui.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include "a_star.h"

using namespace std;
ros::Publisher vel_pub;

double angular = 0.0;
double linear = 0.0;

//odom variables
double posX, posY, yaw;
double pi = 3.1416;

//bumper variables
bool bumperLeft = 0, bumperCenter = 0, bumperRight = 0;

//laser variables
double laserRange = 10;
int laserSize = 0, laserOffset = 0, desiredAngle = 10;
float minAngle = 0.0 , maxAngle = 0.0, incrAngle = 0.0, rangeMax = 0.0, rangeMin = 0.0;

void bumperCallback(const kobuki_msgs::BumperEvent msg){
	if(msg.bumper == 0){
		bumperLeft = !bumperLeft;
	}else if(msg.bumper == 1){
		bumperCenter = !bumperCenter;
	}else if(msg.bumper == 2){
		bumperRight = !bumperRight;
	}
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
	ROS_INFO("Anxious\n");
	laserSize = (msg->angle_max - msg->angle_min)/msg->angle_increment; // Number of range indices in sensor field of view
	laserOffset = desiredAngle*pi/(180*msg->angle_increment); // Number of range indices in desired field of view
	laserRange = 11; // Minimum number of range indices

	if(desiredAngle*pi/180 < msg->angle_max && -desiredAngle*pi/180 > msg->angle_min){ // Desired FOV is smaller than sensor FOV
		for(int i = laserSize/2 - laserOffset; i < laserSize/2 + laserOffset; i++){
			if(laserRange > msg->ranges[i]){
				laserRange = msg->ranges[i];
			}
		}
	}else{ // Desired FOV is larger than sensor FOV
		for(int i = 0; i < laserSize; i++){
			if(laserRange > msg->ranges[i]){
				laserRange = msg->ranges[i];
			}
		}
	}

	if(laserRange == 11){
		laserRange = 0;
	}
	printf("HELLO\n");

	minAngle = msg->angle_min; maxAngle = msg->angle_max; incrAngle = msg->angle_increment;
	rangeMin = msg->ranges[0]; rangeMax = msg->ranges[laserSize];
	ROS_INFO("Size of laser scan array: %i and size offset: %i", laserSize, laserOffset);
	ROS_INFO("Angle Min: %f, Angle Max: %f", msg->angle_min, msg->angle_max);

	ROS_INFO("Size of laser scan array: %i and size offset: %i", laserSize, laserOffset);
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
	posX = msg->pose.pose.position.x;
	posY = msg->pose.pose.position.y;
	yaw = tf::getYaw(msg->pose.pose.orientation);

	ROS_INFO("Position: (%f, %f) Orientation: %f rad or %f degrees.", posX, posY, yaw, yaw*180/pi);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh;
	teleController eStop;

	ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
	ros::Subscriber laser_sub = nh.subscribe("/scan", 10, &laserCallback);
	ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);
	ros::Subscriber odom = nh.subscribe("/odom", 1, odomCallback);

	vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

	angular = 0.0;
	linear = 0.0;
	geometry_msgs::Twist vel;

	while(ros::ok()){
		ros::spinOnce();
		//.....**E-STOP DO NOT TOUCH**.......
		eStop.block();
		//...................................

		//fill with your code
		// ROS_INFO("Position: (%f, %f) Orientation: %f degrees Range: %f", posX, posY, yaw*180/pi, laserRange);
		ROS_INFO("Angle Min: %f, Angle Max: %f, Angle increment: %f, Range min: %f, Range Max: %f", \
			 minAngle, maxAngle, incrAngle, rangeMin, rangeMax);
		ROS_INFO("Position: (%f, %f) Orientation: %f degrees Range: %f", posX, posY, yaw*180/pi, laserRange);

		//Odometry and bumper code
		if(posX < 0.5 && yaw < pi/2 && !bumperRight && !bumperCenter && !bumperLeft && laserRange > 0.7){
			angular = 0.0;
			linear = 0.2;
		}else if(yaw < pi/2 && posX > 0.5 && !bumperRight && !bumperCenter && !bumperLeft && laserRange > 0.5){
			angular = pi/6;
			linear = 0.0;
		}else if(laserRange > 1.0 && !bumperRight && !bumperCenter && !bumperLeft){
			if(yaw < 17*pi/36 || posX > 0.6){
				angular = pi/12;
				linear = 0.1;
			}else if(yaw > 19*pi/36 || posX < 0.4){
				angular = -pi/12;
				linear = 0.1;
			}else{
				angular = 0.0;
				linear =  0.1;
			}
		}else{
			angular = 0.0;
			linear = 0.0;
		}

		// angular = 0.0;
		// linear = 0.0;

		vel.angular.z = angular;
		vel.linear.x = linear;

		vel_pub.publish(vel);
	}

	return 0;
}
