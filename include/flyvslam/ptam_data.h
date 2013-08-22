#ifndef _PTAMDATA_H
#define _PTAMDATA_H

#include <TooN/TooN.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "ros/ros.h"

//Class to store the PTAM data received.
//This class stores n previous PTAM data frames and provides a basic backwards differential velocity.
class ptam_data
{
	public:
		//Constructor
		ptam_data();

		//Update the PTAM info with a new ptam frame.
		void update(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);

		//Set the Vicon position when PTAM initialized
		void setInitVicon(TooN::Vector<3, double> initPos,TooN::Vector<4, double> initRot);

		//Set the PTAM scale
		void setPtamScale(double scaleTemp);



		//Storage for the initial PTAM pose
		TooN::Vector<3, double> initPtamPos;
		TooN::Vector<4, double> initPtamRot;
		TooN::Vector<3, double> initCamerCorr;
		TooN::Vector<3, double> cameraOffset;
		int setPtamInit;
		int setCameraCorr;

		//Storage for the initial Vicon pose when PTAM inits. Store the inverse orientation for ease.
		TooN::Vector<3, double> initViconPos;
		TooN::Vector<4, double> initViconRot;
		TooN::Vector<4, double> initViconRotInv;
		int setViconInit;

		//Current pose info. This is the latest measurement transformed into NED
		TooN::Vector<3, double> currentPos;
		TooN::Vector<3, double> currentVel;
		TooN::Vector<4, double>	currentRot;
		TooN::Vector<3, double> currentEuler;
		double currentYaw;

		//Scale
		double ptamScale;
		
		
		//Storage for the last received ptam data
		int ptamCount;
		int ptamIdx;
		TooN::Vector<3, double> * ptamsPos;
		ros::Time * ptamsTime;
};
#endif
//eof
