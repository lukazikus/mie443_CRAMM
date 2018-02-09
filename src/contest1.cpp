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
double yawStart = 0;
double d = 0.52;

//robot direction
int direction = 0;
float rot = 0;
float RES_ANG = 0.1;


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

	if(dist == 11){
		dist = 0;
	}
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
	posX = msg->pose.pose.position.x;
	posY = msg->pose.pose.position.y;
	yaw = tf::getYaw(msg->pose.pose.orientation);

	ROS_INFO("Position: (%f, %f) Orientation: %f rad or %f degrees.\n", posX, posY, yaw, yaw*180/pi);
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
		//.....**E-STOP DO NOT TOUCH**.......
		eStop.block();
		if(dist > 0.6){
			//drive forward
			printf("fwd\n");
			linear = 0.2;
			angular = 0.0;
			vel.linear.x = linear;
			vel.angular.z = angular;
			vel_pub.publish(vel);
		}
		else{
			//stop, pause, turn
			printf("turn\n");
			linear = 0.0;
			vel.linear.x = linear;
			vel_pub.publish(vel);
			sleep(1);
			
			direction = rand() % 3; //random number 0 or 1
			printf("DIRECTION:%d \n",direction);
			// Detect if the robot needs to change directions
			if(direction == 0){ // Make robot face left
				printf("1111111111111111\n");
				// Change directions
				while(ros::ok() && abs(abs(yaw)-pi) > RES_ANG){
					// printf("CURRENT YAW: %f, DESIRED YAW: %f\n", yaw, pi);
					ros::spinOnce();
					linear = 0.0;
					rot = yaw > 0 && yaw < pi? 1:-1; // Rotate in the direction of shortest angular displacement
					angular = pi/6*rot;
					vel.angular.z = angular;
					vel.linear.x = linear;
					vel_pub.publish(vel);
					ros::spinOnce();
					// usleep(5000);
				}
			}else if(direction == 1){ // Make robot face right
				printf("222222222222222\n");
				printf("hello it's me yaw %f \n", yaw);
				while(ros::ok() && abs(yaw) > RES_ANG){
					ros::spinOnce();
					printf("Stuck facing right\n");
					linear = 0.0;
					rot =  yaw > -pi && yaw < 0 ? 1:-1;
					angular = pi/6*rot;
					vel.angular.z = angular;
					vel.linear.x = linear;
					vel_pub.publish(vel);
					ros::spinOnce();
					// usleep(5000);
				}
			}else if(direction == 2){ // Make robot face down
				printf("3333333333333\n");
				while(ros::ok() && abs(yaw + pi/2) > RES_ANG){
					ros::spinOnce();
					printf("Stuck facing down\n");
					linear = 0.0;
					rot = (yaw > -pi && yaw < -pi/2) || (yaw > pi/2 && yaw < pi) ? 1:-1;
					angular = pi/6*rot;
					vel.angular.z = angular;
					vel.linear.x = linear;
					vel_pub.publish(vel);
					ros::spinOnce();
					// usleep(5000);
				}
			}else if (direction == 3){// Make robot face up
				printf("444444444444\n");
				while(ros::ok() && abs(yaw - pi/2) > RES_ANG){
					ros::spinOnce();
					// printf("Stuck facing up\n");
					printf("CURRENT YAW: %f, DESIRED YAW: %f\n", yaw, pi/2);
					linear = 0.0;
					rot = yaw > -pi/2 && yaw < pi/2 ? 1:-1;
					angular = pi/6*rot;
					printf("Angular velocity: %f\n", angular);
					vel.angular.z = angular;
					vel.linear.x = linear;
					vel_pub.publish(vel);
					ros::spinOnce();
					// usleep(5000);
				}
			}
			// if(direction){
			// 	//rotate right counterclockwise
			// 	yawStart = yaw;
			// 	printf("left yaw, yaw desired: (%f, %f) \n",abs(yaw + pi),abs(yawStart + pi - pi/2));
			// 	while(ros::ok() && abs(yaw + pi) < abs(yawStart + pi + pi/2) ){
			// 		printf("right yaw, yaw desired: (%f, %f) \n",abs(yaw + pi),abs(yawStart + pi - pi/2));
			// 		ros::spinOnce();
			// 		linear = 0.0;
			// 		angular = -pi/6;
			// 		vel.angular.z = angular;
			// 		vel.linear.x = linear;
			// 		vel_pub.publish(vel);
			// 	}
			// 	//stop the robot
			// 	ros::spinOnce();
			// 	linear = 0.0;
			// 	angular = 0.0;
			// 	vel.angular.z = angular;
			// 	vel.linear.x = linear;
			// 	vel_pub.publish(vel);
			// }
			// else{
			// 	//rotate left (assuming -pi/2 is counterclockwise)
			// 	yawStart = yaw;
			// 	//printf("Yaw Value ELSE %f \n",yaw);
			// 	printf("left yaw, yaw desired: (%f, %f) \n",abs(yaw),abs(yawStart - pi/2));
			// 	while(ros::ok() && abs(yaw) > abs(yawStart- pi/2) ){
			// 		printf("left yaw, yaw desired: (%f, %f) \n",abs(yaw,abs(yawStart + pi - pi/2));
					
			// 		//printf("turn left\n");
			// 		ros::spinOnce();
			// 		linear = 0.0;
			// 		angular = pi/6;
			// 		vel.angular.z = angular;
			// 		vel.linear.x = linear;
			// 		vel_pub.publish(vel);
			// 	}
			// 	//stop the robot
			// 	ros::spinOnce();
			// 	linear = 0.0;
			// 	angular = 0.0;
			// 	vel.angular.z = angular;
			// 	vel.linear.x = linear;
			// 	vel_pub.publish(vel);
			// }
		}
		
	}
	return 0;
}

