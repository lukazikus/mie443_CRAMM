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
#include <string>

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
double dist = 11;
int direction = 0;

void bumperCallback(const kobuki_msgs::BumperEvent msg){
	if(msg.bumper == 0){
		bumperLeft = !bumperLeft;
	}else if(msg.bumper == 1){
		bumperCenter = !bumperCenter;
	}else if(msg.bumper == 2){
		bumperRight = !bumperRight;
	}
}

void scanProfile (const sensor_msgs::LaserScan::ConstPtr& msg){ 
	int i;
	double min_angle = msg->angle_min + pi/2;	
	laserSize = (msg->angle_max - msg->angle_min)/msg->angle_increment; // Number of range indices in sensor field of view
	dist = 11;
	
	for (i = 0; i < laserSize; i++) { // loop through each laser scan index
		if(abs(msg->ranges[i] / tan(min_angle + i*msg->angle_increment)) <= d ){
			// laser ranges are within the left square
			if(msg->ranges[i] < dist) {
				dist = msg->ranges[i];
			}
		}
	
	}
	
	printf("Made it");

	if(dist == 11){
		dist = 0;
	}
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
	posX = msg->pose.pose.position.x;
	posY = msg->pose.pose.position.y;
	yaw = tf::getYaw(msg->pose.pose.orientation);

	ROS_INFO("Position: (%f, %f) Orientation: %f rad or %f degrees.", posX, posY, yaw, yaw*180/pi);
}

int main(int argc, char **argv)
{	
	ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh;
	teleController eStop;
	
	ros::Subscriber laser_sub = nh.subscribe("/scan", 10, &scanProfile);
	ros::Subscriber odom = nh.subscribe("/odom", 1, odomCallback);

	vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
	angular = 0.0;
	linear = 0.0;
	geometry_msgs::Twist vel;
	
	while(ros::ok){
		ros::spinOnce();
		if(dist > 2.0){
			//drive forward
			linear = 0.2;
			angular = 0.0;
			vel.linear.x = linear;
			vel.angular.z = angular;
			vel_pub.publish(vel);
		}
		else{
			//stop, pause, turn
			linear = 0.0;
			vel.linear.x = linear;
			vel_pub.publish(vel);
			sleep(1);
			
			direction = rand() % 1; //random number 0 or 1
			if(direction){
				//rotate right (assuming +pi/2 is clockwise?)
				yawStart = yaw;
				while(ros::ok() && yaw < yawStart + pi/2 ){
					ros::spinOnce();
					linear = 0.0;
					angular = 0.1;
					vel.angular.z = angular;
					vel.linear.x = linear;
					vel_pub.publish(vel);
				}
			}
			else{
				//rotate left (assuming -pi/2 is counterclockwise)
				yawStart = yaw;
				while(ros::ok() && yaw < yawStart - pi/2 ){
					ros::spinOnce();
					linear = 0.0;
					angular = -0.1;
					vel.angular.z = angular;
					vel.linear.x = linear;
					vel_pub.publish(vel);
				}
			}
		}
		
	}
	return 0;
}
