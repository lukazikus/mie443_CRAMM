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
#include <iostream>
#include <algorithm>
#include <vector>

using namespace std;
ros::Publisher vel_pub;

double angular = 0.0;
double linear = 0.0;

//odom variables
double posX, posY, yaw;
double pi = 3.1416;
geometry_msgs::Twist vel;
int incr_ang = 40; // Increments that the robot should travel when scanning distance in degrees
int size_yaw_values = 360/incr_ang; // Number of yaw values that the robot should go to

vector<float> yaw_values(size_yaw_values); // Different yaw values (in radians) that the robot should stop to scan an area in
vector<float> dist_config(size_yaw_values); // Different distance measurements to keep track of
float yaw_check = 0.0; // Next yaw value that the robot should rotate to for checking
float yaw_best = 0.0; // Best yaw value that the robot should rotate to after discerning the best one
int max_distance_index = 0; // Index of maximum distance
vector<float>::iterator largest;

//bumper variables
bool bumperLeft = 0, bumperCenter = 0, bumperRight = 0;

//laser variables
double laserRange = 10;
int laserSize = 0, laserOffset = 0, desiredAngle = 10;
double dist = 11;
double yawStart = 0;
double d = 0.52;
int wall_config[4] = {0, 0, 0, 0};
float max_distance = 0.0;
float *max_distances;

//robot direction
int direction = 0;
float rot = 0;
float RES_ANG = 0.1;
int coin = 0;

void robot_rotate(){
	for(int i=0; i<size_yaw_values; i++){ // Scan 360 for best direction to go in next
		yaw_check = yaw_values.at(i); // Extract yaw value to go to
		linear = 0.0;
		angular = pi/6;
		vel.angular.z = angular;
		vel.linear.x = linear;
		while(ros::ok() && abs(yaw - yaw_check) > RES_ANG){
			ros::spinOnce();
			printf("CURRENT YAW: %f, DESIRED YAW: %f\n", yaw, yaw_check);
			vel_pub.publish(vel);
			// usleep(5000);
		}
		ros::spinOnce();
		if (dist > 10){
			dist_config.at(i) = 10;
		}else{
			dist_config.at(i) = dist; // Extract distance value of current yaw angle
		}
		
		
		linear = 0.0;
		angular = 0.0;
		vel.angular.z = angular;
		vel.linear.x = linear;
		vel_pub.publish(vel);
		usleep(5000);
		printf("Done current angle of %f\n", yaw_check);
	}

	printf("First distance, Last distance: %f, %f\n", dist_config.begin(), dist_config.end());

	largest = max_element(dist_config.begin(), dist_config.end());
	max_distance = *largest; // Extract maximum yaw value
	max_distance_index = distance(dist_config.begin(), largest); // Extract index of maximum value
	yaw_best = yaw_values.at(max_distance_index); // Extract best yaw value to rotate to

	printf("Best yaw is: %f, largest is %f \n", yaw_best, largest);

	//Rotate the robot to the best yaw
	ros::spinOnce();
	linear = 0.0;
	angular = pi/6;
	vel.angular.z = angular;
	vel.linear.x = linear;

	while(ros::ok() && abs(yaw - yaw_best) > RES_ANG){
		ros::spinOnce();
		//printf("ROTATING TO BEST YAW: CURRENT YAW: %f, DESIRED YAW: %f\n", yaw, yaw_best);
		vel_pub.publish(vel);
		// usleep(5000);
	}
	ros::spinOnce();
	linear = 0.0;
	angular = 0.0;
	vel.angular.z = angular;
	vel.linear.x = linear;
	vel_pub.publish(vel);
	usleep(5000);
}

void bumperCallback(const kobuki_msgs::BumperEvent msg){
	if(msg.bumper == 0 || msg.bumper == 1 || msg.bumper == 2){
		ros::spinOnce();
		usleep(5000);
		linear = -0.2;
		angular = 0.0;
		vel.angular.z = angular;
		vel.linear.x = linear;
		vel_pub.publish(vel); // Stop the robot after one grid is surpassed
		ros::spinOnce();
		usleep(5000);
		coin = rand() % 2; //random number 0 or 1
		if (coin == 1){
			direction += 1;
		}else{
			direction -= 1;
		}
		robot_rotate();
		if(msg.bumper == 0){
			bumperLeft = !bumperLeft;
		}else if(msg.bumper == 1){
			bumperCenter = !bumperCenter;
		}else if(msg.bumper == 2){
			bumperRight = !bumperRight;
		}
	}
}

void scanProfile (const sensor_msgs::LaserScan::ConstPtr& msg){ 
	int i;
	double min_angle = msg->angle_min + pi/2;	
	laserSize = (msg->angle_max - msg->angle_min)/msg->angle_increment; // Number of range indices in sensor field of view
	dist = 11; // In centimetres
	
	for (i = 0; i < laserSize; i++) { // loop through each laser scan index
		if(abs(msg->ranges[i] / tan(min_angle + i*msg->angle_increment)) <= d ){
			// laser ranges are within the left square
			if(msg->ranges[i] < dist) {
				dist = msg->ranges[i];
			}
		}
	}

	if(dist == 11){
		dist = 0;
	}
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
	posX = msg->pose.pose.position.x;
	posY = msg->pose.pose.position.y;
	yaw = tf::getYaw(msg->pose.pose.orientation);

	//ROS_INFO("Position: (%f, %f) Orientation: %f rad or %f degrees.\n", posX, posY, yaw, yaw*180/pi);
}


int main(int argc, char **argv){
	ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh;
	teleController eStop;
	
	ros::Subscriber laser_sub = nh.subscribe("/scan", 10, &scanProfile);
	ros::Subscriber odom = nh.subscribe("/odom", 1, odomCallback);

	vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
	angular = 0.0;
	linear = 0.0;

	for(int i = 0; i < size_yaw_values; i++){
		yaw_values.at(i) = (i*incr_ang-180)*pi/180;
	}
	
	while(ros::ok()){
		ros::spinOnce();
		//.....**E-STOP DO NOT TOUCH**.......
		eStop.block();
		if(dist > 0.5){
			//drive forward
			//printf("fwd\n");
			if (dist < 0.8){ 
				linear = 0.1;
			}else{
				linear = 0.25;
			}
			angular = 0.0;
			vel.linear.x = linear;
			vel.angular.z = angular;
			vel_pub.publish(vel);
		}else{
			// Clear all distance measurements
			for (int j = 0; j<4; j++){
				dist_config.at(j)= 0;
			}
			
			// Stop robot
			linear = 0.0;
			vel.linear.x = linear;
			vel_pub.publish(vel);
			sleep(1);

			robot_rotate(); // Rotate robot to align in next best direction
		}
	}
	return 0;
}