#ifndef _WAYPOINTDATA_H
#define _WAYPOINTDATA_H

#include <TooN/TooN.h>
#include "ros/ros.h"


///Class to store the waypoint data.
///This class reads a list of waypoint data from file into a storage structure, then provides indexed access to the waypoint data.
class waypoint_data
{
	public:
		///Empty constructor
		waypoint_data();

		//Get target position and yaw depending on the current time. The start_time must be set at the beginning of the flight.
		//This function then determines the current target position and yaw depending upon the passed timeNow.
		double getTargetYaw();
		TooN::Vector<3, double> getTargetPos(ros::Time timeNow);

		//Function to read all data from file. Assumes "waypoints.txt" is in the same dir.
		bool readWaypointData();


		//Total number of waypoints read from file.
		int waypointCount;

		//Total number of times waypoints have been flown
		int waypointLoops;

		//Pointer to a list of waypoint positions, look points, and durations
		//This is the main storage structure.
		TooN::Vector<3, double> * waypointsPos;
		TooN::Vector<3, double> * waypointsLook;
		ros::Duration * waypointsTime;

		//Current waypoint index. The getTagetYaw and getTargetPos functions lerp between the current index and the next index.
		int currentIdx;

		//Clock time the last waypoint changed, and start of the waypoint following flight.
		ros::Time waypoint_time;
		ros::Time start_time;

		//Storage for the current lerp'd targetPos and targetLook
		TooN::Vector<3, double> targetPos;
		TooN::Vector<3, double> targetLook;
};

#endif
//eof
