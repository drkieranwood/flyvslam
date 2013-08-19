#ifndef _VICONDATA_H
#define _VICONDATA_H

#include <TooN/TooN.h>
#include "geometry_msgs/TransformStamped.h"
#include "ros/ros.h"

///Class to store the Vicon data received.
///This class stores n previous vicon data frames and provides a differential velocity.
class vicon_data
{
	public:
		//Empty constructor.
		vicon_data();

		//Calculate velocity from previous frames.
		double getViconVel();

		//Update the Vicon info with a new vicon frame.
		void update(const geometry_msgs::TransformStamped::ConstPtr& msg);


		//Number of Vicon frames to use in the velocity estimation.
		int viconCount;

		//Storage for latest received Vicon data
		TooN::Vector<3, double> * viconsPos;
		TooN::Vector<4, double> * viconsRot;
		ros::Time * viconsTime;

		//Current index within the storage array.
		int viconIdx;
		ros::Time vicon_time;

		//The last time a Vicon frame was received. This is used to check for droputs and hence halt the contoller.
		ros::Time vicon_last_update_time;
		
		//Latest calculated Vicon velocity (this is used in the control). These values are converted into the NED RF.
		TooN::Vector<3, double> currentVel;
		TooN::Vector<3, double> currentPos;
		double currentYaw;
		TooN::Vector<3, double> currentEuler;
		TooN::Vector<4, double> currentRot;
		
};
#endif
//eof
