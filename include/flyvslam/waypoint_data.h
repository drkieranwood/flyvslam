#ifndef _WAYPOINTDATA_H
#define _WAYPOINTDATA_H

#include <TooN/TooN.h>
#include "ros/ros.h"


//Class to store the waypoint data.
//This class reads a list of waypoint data from file into a storage structure, 
//then provides access to the waypoint data.
class waypoint_data
{
	public:
		//Constructor
		waypoint_data();

		//Get target position and yaw depending on the current time.
		//The start_time must be set at the beginning of the flight (or whenever 
		//the waypoints should start.
		//This function then determines the current target position and 
		//yaw depending upon the passed argument timeNow (the current time).
		//The target yaw is updated after a call to getTargetPos(), hence always 
		//call getTargetYaw() immidiately after getTargetPos().
		double getTargetYaw(void);
		TooN::Vector<3, double> getTargetPos(ros::Time timeNow);

		//Function to read all data from file. Assumes "waypoints.txt" is 
		//in the same dir. Ignores blank lines and lines starting with a 
		//hash (#). Returns true(1) if success or 0 elsewise.
		bool readWaypointData(void);


		//Total number of waypoints read from the file.
		int waypointCount;

		//Pointer to a list of waypoint positions, look points, and durations
		//This is the main storage structure. The actual correctly sized arrays 
		//are created in the readWaypointData() function.
		TooN::Vector<3, double> * waypointsPos;
		TooN::Vector<3, double> * waypointsLook;
		ros::Duration * waypointsTime;

		//Current waypoint index (i.e. the last waypoint passed/reached)
		//The getTagetYaw() and getTargetPos() functions lerp between the 
		//current index and the next index over the specified duration time.
		int currentIdx;

		//Clock time the last waypoint changed, and start of the whole 
		//waypoint following flight.
		ros::Time waypoint_time;
		ros::Time start_time;

		//Storage for the current lerp'd targetPos and targetLook positions. 
		//Also store the target yaw for checking of singularities.
		TooN::Vector<3, double> targetPos;
		double targetYaw;
		TooN::Vector<3, double> targetLook;
};

#endif
//eof
