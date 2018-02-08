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
#include <unistd.h>

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

//Mapping Variables
int robot_i = 24;
int robot_j = 24;
int BlocksToScan = 5; // defines the depth to scan in front of the robot
double scale = 0.52;// defines the scale of the array based on 1/3 the robot size
int robotHeading = 0; // define 0 = up, 1=left, 2 = down, 3 = right
char maze_input[ROW][COL];// 'U' = unexplored, 'O' = unblocked, 'B' = blocked, 'T' = travelled
float d = 0.52; // size of the robot

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
	double min_angle = msg->angle_min + pi/2;	//double distArray[3]; // HAS TO BE GLOBAL
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

//-------------------------------Mapping Scan Functions------------------------------------------

void upscan(Pair src){
	for(int j=1; j<= BlocksToScan; j++){ // Depth of scan in blocks
		if (src.first -j >= 0 && maze_input[src.first - j][src.second] != 'T'){ // If the point is not outside the array or already travelled
			if (dist - scale*j > 0){
				maze_input[src.first - j][src.second] = 'O';
			}
			else {
				maze_input[src.first - j][src.second] = 'B';
				break;
			}
		}
		else{
			break;
		}
	}
}
	
void leftscan(Pair src){
	for(int j=1; j<= BlocksToScan; j++){ // Depth of scan in blocks
		if (src.second-j > 0 && maze_input[src.first][src.second - j] != 'T'){ // If the point is not outside the array or already travelled
			if (dist - scale*j > 0){
				maze_input[src.first][src.second - j] = 'O';
			}
			else {
				maze_input[src.first][src.second - j] = 'B';
				break;
			}
		}
		else{
			break;
		}
	}
}
	
void downscan(Pair src){
	for(int j=1; j<= BlocksToScan; j++){ // Depth of scan in blocks
		if (src.first + j < ROW && maze_input[src.first + j ][src.second] != 'T'){ // If the point is not outside the array or already travelled
			if (dist - scale*j > 0){
				maze_input[src.first + j][src.second] = 'O';
		}
			else {
				maze_input[src.first + j][src.second] = 'B';
				break;
			}
		}
		else{
			break;
		}
	}
}
void rightscan(Pair src){
	for(int j=1; j<= BlocksToScan; j++){ // Depth of scan in blocks
		printf("Source: %d, %d\n", src.first, src.second);
		if (src.second + j < COL && maze_input[src.first][src.second+j] != 'T'){ // If the point is not outside the array or already travelled
			if (dist - scale*j > 0){
				
				maze_input[src.first][src.second + j] = 'O';
				printMap_input(maze_input);
				// printf("");
			}else {
				maze_input[src.first][src.second + j] = 'B';
				printMap_input(maze_input);
				break;
			}
		}
		else{
			break;
		}
	}
}
void UpdateMap(Pair src){
	cout << dist << endl;
	// maze_input[robot_i][robot_j] = 'T';
	if (dist != 11) {
		if (yaw  > pi/4 && yaw <= 3*pi/4 ) {
			upscan(src);
			printf("UPSCANNING\n");
		}else if (yaw > 3*pi/4 || yaw <= -3*pi/4){
			leftscan(src);
			printf("LEFTSCANNING\n");
		}else if (yaw < -pi/4 && yaw > -3*pi/4){
			downscan(src);
			printf("DOWNSCANNING\n");
		}else if (yaw < pi/4 && yaw > -pi/4){
			printf("Src: %d, %d\n", src.first, src.second);
			rightscan(src);
			printf("RIGHTSCANNING\n");
		}
	}
}

//----------------------Goal Function------------------------------------
Pair goal( Pair cur_pos){
    int direction;//0: forward, 1: left, 2: down, 3: right
    // Maze inputs are 0: unknown, 1: unblocked, 2: blocked, 3: travelled
    Pair rob_dest; // Robot destination
    Pair obs_dest; // Obstacle destination
    Pair final_dest; // Can eventually be same as rob_dest
    final_dest.first = -1;
    final_dest.second = -1;
	// Only find the shortest U at the front
	
	printf("%d,%d", cur_pos.first,cur_pos.second);
	

    for (int i = cur_pos.first - 1; i >= 0; i--){
        if (maze_input[i][cur_pos.second] != 'T'){//first O at the front
            if (maze_input[i][cur_pos.second] == 'O'){
                final_dest = make_pair(i, cur_pos.second);
                direction = 0;
                return final_dest;
            }
            break;
        }
    }
    // Only find the shortest U at the right
    for (int i = cur_pos.second + 1; i < COL; i++){
		printf("%d", i);
        if (maze_input[cur_pos.first][i] != 'T'){//first O at the right
            if (maze_input[cur_pos.first][i] == 'O'){
                final_dest = make_pair(cur_pos.first, i);
                direction = 3;
                return final_dest;
            }
            break;
        }
    }
    // Only find the shortest U at the left
    for (int i = cur_pos.second - 1; i >= 0; i--){
        if (maze_input[cur_pos.first][i] != 'T'){//first O at the front
            if (maze_input[cur_pos.first][i] == 'O'){
                final_dest = make_pair(cur_pos.first, i);
                direction = 1;
                return final_dest;
            }
            break;
        }
    }
    // Only find the shortest U at the back
    for (int i = cur_pos.first + 1; i < ROW; i++){
        //maze_input[cur_pos.first][i] == 0 || maze_input[cur_pos.first][i] == 2 || maze_input[cur_pos.first][i] == 3 
        if (maze_input[i][cur_pos.second] != 'T') {//first U at the front
            if (maze_input[i][cur_pos.second] == 'O'){
                final_dest = make_pair(i, cur_pos.second);
                direction = 2;
                return final_dest;
            }
            break;
        }
    }

    return final_dest;
}

int main(int argc, char **argv){
	for(int i=0; i<ROW; i++){
		for (int j=0; j<COL; j++){
			maze_input[i][j] = 'U';
		}
	}
	maze_input[ROW/2][COL/2] = 'D';

	ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh;
	teleController eStop;

	ros::spinOnce();
	ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
	ros::Subscriber laser_sub = nh.subscribe("scan", 10, &scanProfile);
	ros::Subscriber odom = nh.subscribe("/odom", 1, odomCallback);

	vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

	angular = 0.0;
	linear = 0.0;
	geometry_msgs::Twist vel;

	// Hard-coded maze_input (will be replaced by Navigation node's output)
	int grid[ROW][COL];
	Pair src, dest; // Robot's initial position
	src = make_pair(ROW/2,COL/2);

	stack<Pair> Path;
	int wayX, wayY; // Keep track of robot's current goal coordinates in the grid frame

	// HARD CODED MAZE
	/*
	char maze_input[ROW][COL] = {{'B','B','B','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','B','B','B','B','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'B','B','B','B','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','B','B','B','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'B','B','B','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','B','B','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'U','B','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','B','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','U','U','U','U','U','U','U','U','B','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','U','U','U','U','U','U','U','B','B','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','U','U','U','U','U','U','B','B','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','U','U','U','U','U','B','B','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','U','U','U','U','B','B','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','U','U','U','B','B','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','U','U','B','B','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','U','B','B','U','U','U','U','U','U','U','U','U','U','U','U','U','B','B','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','B','B','U','U','U','U','U','U','U','U','U','U','U','U','U','U','B','B','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','B','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','B','B','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','B','B','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','B','B','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','B','B','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','B','B','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','B','B','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','B','B','B','B','B','B','B','B','B','B','B','B','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','B','B','B','B','B','B','B','B','B','B','B','B','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','B','B','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','B','B','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','B','B','U','U','U','U','U','U','U','B','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','B','B','U','U','U','U','U','U','B','B','B','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','U','U','U','U','U','U','U','B','B','B','B','B','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'O','O','O','U','U','U','U','U','U','U','U','U','U','B','B','B','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'O','O','O','U','U','U','U','U','U','U','U','U','U','U','B','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'U','O','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'U','W','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
                               {'U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
				{'U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
				{'U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
				{'U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
				{'U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
				{'U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
				{'U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
				{'U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
				{'U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
				{'U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
				{'U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
				{'U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
				{'U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
				{'U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
				{'U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
				{'U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
				{'U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'},
				{'U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U','U'}};
	*/

	int dir = 0; // 0 means forward, 1 means backward, 2 means left, 3 means right
	int rot = 1; // 1 means CCW, -1 means CW
	float nextX,nextY = 0.0;
	double delta_time = GRID_WIDTH / VEL; // Calculate time to travel one grid block

	// ROBOT MUST FACE RIGHT AT THE BEGINNING
	sleep(5);
	while(ros::ok()){
		ros::spinOnce();
		//.....**E-STOP DO NOT TOUCH**.......
		eStop.block();
		//...................................

		// Sensor information display
		ROS_INFO("Position: (%f, %f) Orientation: %f degrees Range: %f", posX, posY, yaw*180/pi, laserRange);

		
		UpdateMap(src); // Update map to get the next path to follow
		//printMap_input(maze_input); // Print map

		convertMap(maze_input, grid, src); // Create grid map that can be parsed by A*, extract current location
		dest = goal(src); // Determine next destination for robot to follow

		printf("(SrcX,SrcY) = (%d, %d)\n", src.second,src.first);
		printf("(DestX,DestY) = (%d, %d)\n", dest.second,dest.first);
		
		aStarSearch(grid, src, dest, Path); // Compute path
		tracePath(Path);

		while(!Path.empty()){ // Follow the current path's goal waypoints
			Pair p = Path.top(); // Extract current waypoint
	       	Path.pop(); // Remove waypoint from stack
			wayX = p.second; // Calculate next coordinates of the robot
			wayY = p.first;

			// printf("WayX,WayY = (%d,%d)\n", wayX, wayY);
			// printf("(CurX,CurY,Yaw) = (%d, %d, %f)\n", src.second, src.first, yaw);

			// Detect if the robot needs to change directions
			if(wayX < src.second){ // Make robot face left
				printf("1111111111111111\n");
				dir = 2;
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
					// usleep(5000);
				}
			}else if(wayX > src.second){ // Make robot face right
				printf("11111112221111\n");
				dir = 3;
				while(ros::ok() && abs(yaw) > RES_ANG){
					ros::spinOnce();
					printf("Stuck facing right\n");
					linear = 0.0;
					rot =  yaw > -pi && yaw < 0 ? 1:-1;
					angular = pi/6*rot;
					vel.angular.z = angular;
					vel.linear.x = linear;
					vel_pub.publish(vel);
					// usleep(5000);
				}
			}else if(wayY > src.first){ // Make robot face down
				printf("11111113331111\n");
				dir = 1;
				while(ros::ok() && abs(yaw + pi/2) > RES_ANG){
					ros::spinOnce();
					printf("Stuck facing down\n");
					linear = 0.0;
					rot = (yaw > -pi && yaw < -pi/2) || (yaw > pi/2 && yaw < pi) ? 1:-1;
					angular = pi/6*rot;
					vel.angular.z = angular;
					vel.linear.x = linear;
					vel_pub.publish(vel);
					// usleep(5000);
				}
			}else if (wayY < src.first){// Make robot face up
				printf("1111444111111111\n");
				dir = 0;
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
					// usleep(5000);
				}
			}
			// Move forward to the next waypoint after the robot orients in the correct direction

			nextX = posX; nextY = posY;
			if(dir == 0) nextY = posY + GRID_WIDTH;
			if(dir == 1) nextY = posY - GRID_WIDTH;
			if(dir == 2) nextX = posX - GRID_WIDTH;
			if(dir == 3) nextY = posY + GRID_WIDTH;
			
			// double begin = ros::Time::now().toSec();
			
			/*while(ros::ok() && ros::Time::now().toSec() - begin < delta_time){ // BUGGY TIME CODE
				ros::spinOnce();
				double d = ros::Time::now().toSec() - begin; 
				printf("Begin Time: %lf, Current Time: %lf, Change: %lf, Delta_time: %lf \n", begin, ros::Time::now().toSec(), d, delta_time);
				linear = 0.2;
				angular = 0.0;
				vel.angular.z = angular;
				vel.linear.x = linear;
				vel_pub.publish(vel);
				// usleep(5000);
			}*/
			
			printf("(POSX, POSY),(%f,%f), (%f, %f)\n", posX, posY, nextX, nextY);
			while(ros::ok() && abs(posX - nextX + posY - nextY) > RES){
				ros::spinOnce();
				printf("(POSX, POSY),(nextX, nextY) = (%f,%f), (%f, %f)\n", posX, posY, nextX, nextY);
				linear = 0.2;
				angular = 0.0;
				vel.angular.z = angular;
				vel.linear.x = linear;
				vel_pub.publish(vel);
				ros::spinOnce();
			}
			
			printf("Reached***************************************************\n");
			linear = 0.0;
			angular = 0.0;
			vel.angular.z = angular;
			vel.linear.x = linear;
			vel_pub.publish(vel); // Stop the robot after one grid is surpassed
			usleep(5000); // Pause after traversing one grid
			// break;
		}
		printf("BROKE OUT OF STACK\n");
		src.first = dest.first; src.second = dest.second; // Update starting location of robot in grid
	}

	return 0;
}
