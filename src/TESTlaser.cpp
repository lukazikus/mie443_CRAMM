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
double distArray = {0, 0, 0};

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

	float d = 0.48; // size of the robot
	double lDist = 11; // Maximum distance the sensor can scan is 10 meters, so this is initialized to 11
	double mDist = 11;
	double rDist = 11;
	bool lFlag = false;
	bool mFlag = false;
	bool rFlag = false;
	//double distArray[3]; // HAS TO BE GLOBAL
	
	laserSize = (msg->angle_max - msg->angle_min)/msg->angle_increment; // Number of range indices in sensor field of view
	
	for (i = 0; i < laserSize; i++) { // loop through each laser scan index
	
		if(msg->ranges[i] / tan(msg->angle_min + i*msg->angle_increment) =< d/2 &&
		msg->ranges[i] / tan(msg->angle_min + i*msg->angle_increment) => d/6){
			// laser ranges are within the left square
			if(lFlag == flase){ // first range becomes minimum range
				lFlag = true;
				lDist = msg->ranges[i];
			}
			elseif(msg->ranges[i] < lDist){ // update minimum distance if we see a smaller distance than we have already recorded 
				lDist = msg->ranges[i];
			}
		}
		
		elseif (msg->ranges[i] / tan(msg->angle_min + i*msg->angle_increment) =< d/6 &&
		msg->ranges[i] / tan(msg->angle_min + i*msg->angle_increment) => -d/6){
			// laser ranges are within the middle square
			if(mFlag == flase){ // first range becomes minimum range
				mFlag = true;
				mDist = msg->ranges[i];
			}
			elseif(msg->ranges[i] < mDist){ // update minimum distance if we see a smaller distance than we have already recorded 
				mDist = msg->ranges[i];
			}
		}
		
		elseif (msg->ranges[i] / tan(msg->angle_min + i*msg->angle_increment) =< -d/6 &&
		msg->ranges[i] / tan(msg->angle_min + i*msg->angle_increment) => -d/2){
			// laser ranges are within the right square
			if(rFlag == flase){ // first range becomes minimum range
				rFlag = true;
				rDist = msg->ranges[i];
			}
			elseif(msg->ranges[i] < rDist){ // update minimum distance if we see a smaller distance than we have already recorded 
				rDist = msg->ranges[i];
			}
		}
	}
	
	if(lDist == 11){
		lDist = 0;
	}
		
	if(mDist == 11){
		mDist = 0;
	}
	
	if(rDist == 11){
		rDist = 0;
	}

	distArray[0] = lDist;
	distArray[1] = mDist;
	distArray[2] = rDist;
	
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
	
	ros::Subscriber laser_sub = nh.subscriber("/scan", 10, &scanProfile);
	
	while(true){
		ROS_INFO("dsitArray = [", distArray[0], ", ", distArray[1], ", ", distArray[2], "]"); //prints: dsitArray = [ x, y, z]
	}
	return 0;
}
