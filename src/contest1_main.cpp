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
#include <a_star.h>

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

	ROS_INFO("Size of laser scan array: %i and size offset: %i", laserSize, laserOffset);
	ROS_INFO("Min Angle: %f, Max Angle: %f", msg->angle_min, msg->angle_max);
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

	ros::spinOnce();
	ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
	ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);
	ros::Subscriber odom = nh.subscribe("/odom", 1, odomCallback);

	vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

	angular = 0.0;
	linear = 0.0;
	geometry_msgs::Twist vel;

	// Hard-coded maze_input (will be replaced by Navigation node's output)
	int grid[ROW][COL];
	Pair src, dest; // Robot's initial position
	stack<Pair> Path;
	int wayX, wayY; // Keep track of robot's current goal coordinates in the grid frame

	// Test input to be replaced by Mapping
	char maze_input_big[ROW*2][COL*2];
    for (int i = 0; i < ROW * 2; i++){
        for (int j = 0; j < COL * 2; j++){
            if (i == ROW && j == COL){
                maze_input_big[i][j] = 'W';
            }else{
                maze_input_big[i][j] = 'U';
            }
        }
    }

	char maze_input[ROW][COL] = {{'B','B','B','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','B','B','B','B'},
                               {'B','B','B','B','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','B','B','B'},
                               {'B','B','B','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','B','B'},
                               {'U','B','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','B'},
                               {'U','U','U','U','U','U','U','U','U','U','U','U','U','B','U','U','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','U','U','U','U','U','U','U','B','B','U','U','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','U','U','U','U','U','U','B','B','U','U','U','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','U','U','U','U','U','B','B','U','U','U','U','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','U','U','U','U','B','B','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','U','U','U','B','B','U','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','U','U','B','B','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','U','B','B','U','U','U','U','U','U','U','U','U','U','U','U','U','B','B'},
                               {'U','U','U','U','U','B','B','U','U','U','U','U','U','U','U','U','U','U','U','U','U','B','B'},
                               {'U','U','U','U','U','B','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','B','B'},
                               {'U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','B','B'},
                               {'U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','B','B'},
                               {'U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','B','B'},
                               {'U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','B','B'},
                               {'U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','B','B'},
                               {'U','U','U','U','U','B','B','B','B','B','B','B','B','B','B','B','B','U','U','U','U','U','U'},
                               {'U','U','U','U','U','B','B','B','B','B','B','B','B','B','B','B','B','U','U','U','U','U','U'},
                               {'U','U','U','U','U','B','B','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','B','B','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','B','B','U','U','U','U','U','U','U','B','U','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','B','B','U','U','U','U','U','U','B','B','B','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','U','U','U','U','U','U','U','B','B','B','B','B','U','U','U','U','U','U'},
                               {'O','O','O','U','U','U','U','U','U','U','U','U','U','B','B','B','U','U','U','U','U','U','U'},
                               {'O','O','O','U','U','U','U','U','U','U','U','U','U','U','B','U','U','U','U','U','U','U','U'},
                               {'T','T','T','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'T','W','T','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'T','T','T','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'}};

	int dir = 0; // 0 means forward, 1 means left, 2 means right
	int rot = 1; // 1 means CCW, -1 means CW

	// ROBOT MUST FACE RIGHT AT THE BEGINNING
	while(ros::ok()){
		ros::spinOnce();
		//.....**E-STOP DO NOT TOUCH**.......
		eStop.block();
		//...................................

		// Sensor information display
		ROS_INFO("Position: (%f, %f) Orientation: %f degrees Range: %f", posX, posY, yaw*180/pi, laserRange);

		convertMap(maze_input, grid, src); // Create grid map that can be parsed by A*// Create grid map that can be parsed by A*
		dest = goal(maze_input, grid, src); // Determine next destination for robot to follow

		printf("(DestX,DestY) = (%d, %d)\n", dest.first,dest.second);

		aStarSearch(grid, src, dest, Path); // Compute path
		tracePath(Path);
		while(!Path.empty()){ // Follow the current path's goal waypoints
			Pair p = Path.top(); // Extract current waypoint
	       	Path.pop(); // Remove waypoint from stack
			wayX = p.second; // Calculate next world coordinates of the robot
			wayY = p.first;

			printf("WayX,WayY = (%d,%d)\n", wayX, wayY);
			printf("(CurX,CurY,Yaw) = (%d, %d, %f)\n", src.first, src.second, yaw);

			// Detect if the robot needs to change directions
			if(wayX < src.second){ // Make robot face left
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
				}
			}else if(wayX > src.second){ // Make robot face right
				while(ros::ok() && abs(yaw) > RES_ANG){
					ros::spinOnce();
					printf("Stuck facing right\n");
					linear = 0.0;
					rot =  yaw > -pi && yaw < 0 ? 1:-1;
					angular = pi/6*rot;
					vel.angular.z = angular;
					vel.linear.x = linear;
					vel_pub.publish(vel);
				}
			}else if(wayY > src.first){ // Make robot face down
				while(ros::ok() && abs(yaw + pi/2) > RES_ANG){
					ros::spinOnce();
					printf("Stuck facing down\n");
					linear = 0.0;
					rot = (yaw > -pi && yaw < -pi/2) || (yaw > pi/2 && yaw < pi) ? 1:-1;
					angular = pi/6*rot;
					vel.angular.z = angular;
					vel.linear.x = linear;
					vel_pub.publish(vel);
				}
			}else if (wayY < src.first){// Make robot face up
				while(ros::ok() && abs(yaw - pi/2) > RES_ANG){// I DID NOT MODIFY WHATEVER IS IN THIS WHILE LOOP + ELSE IF STATEMENT
					ros::spinOnce();
					// printf("Stuck facing up\n");
					// printf("CURRENT YAW: %f, DESIRED YAW: %f\n", yaw, pi);
					linear = 0.0;
					rot = yaw > -pi/2 && yaw < pi/2 ? 1:-1;
					angular = pi/6*rot;
					printf("Angular velocity:");
					vel.angular.z = angular;
					vel.linear.x = linear;
					vel_pub.publish(vel);
					ros::spinOnce();
				}
			}
			// Move forward to the next waypoint after the robot orients in the correct direction
			while(abs(posX-wayX) > RES && abs(posY-wayY > RES)){
				ros::spinOnce();
				printf("Stuck moving\n");
				linear = 0.2;
				angular = 0.0;
				vel.angular.z = angular;
				vel.linear.x = linear;
				vel_pub.publish(vel);
			}
		}
	}

	return 0;
}
