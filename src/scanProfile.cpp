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
