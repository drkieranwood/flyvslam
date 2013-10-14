#ifndef _PTAMDATA_H
#define _PTAMDATA_H

#include <TooN/TooN.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "ros/ros.h"

//Class to store the PTAM data received, and perform scaling and 
//orientation corrections.
//This class stores n previous PTAM data frames and provides a basic 
//backwards differential velocity, averaged over the n frames.
class ptam_data
{
	public:
		//Constructor
		ptam_data();

		//Update the PTAM info with a new ptam frame. 
		//This is the subscriber callback function within a class.
		void update(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);

		//Set the Vicon position when PTAM initialized.
		//If Vicon is used for position and orientation corrections.
		void setInitVicon(TooN::Vector<3, double> initPos,TooN::Vector<4, double> initRot);

		//Set the PTAM scale.
		void setPtamScale(double scaleTemp);
		
		//Set the ptam orientation correction. This is the one that can come from any source.
		void setInitGround(TooN::Vector<3, double> tempPos,TooN::Vector<4, double> tempRot);
		
		//Function to allow the ground orientation correction to be altered during flight.
		void setGroundOrientation(TooN::Vector<4, double> tempRot);
		
		//Storage for the initial PTAM pose.
		TooN::Vector<3, double> initPtamPos;
		TooN::Vector<4, double> initPtamRot;
		TooN::Vector<3, double> initCamerCorr;
		TooN::Vector<3, double> cameraOffset;
		int setPtamInit;       //flag to indicate the initial PTAM pose has been captured.
		int setCameraCorr;	   //flag to indicate the initial camera correction vector has been calculated.

		//Storage for the initial Vicon pose when PTAM inits. Store the inverse orientation for ease.
		TooN::Vector<3, double> initViconPos;
		TooN::Vector<4, double> initViconRot;
		TooN::Vector<4, double> initViconRotInv;
		int setViconInit;      //flag to indicate the initial Vicon pose has been captured.

		//Current pose info. This is the latest measurement transformed into NED.
		//These are the values the control feddback should use.
		TooN::Vector<3, double> currentPos;
		TooN::Vector<3, double> currentVel;
		TooN::Vector<4, double>	currentRot;    //quaternion
		TooN::Vector<3, double> currentEuler;  //Euler triplet in radians
		double currentYaw;

		//The ptam scale correction.
		double ptamScale;
		
		//Flag to indicate at the velocity estimation should start
		int velEst_on;
		
		//These are ground thruth values used to allow a pose correction to the 
		//PTAM raw output.
		//The ptam orientation correction as a quaternion. 
		TooN::Vector<4, double> groundOrientation;
		TooN::Vector<4, double> groundOrientationInv;
		//The ptam position correction as a vector
		TooN::Vector<3, double> groundPosition;
		int setGroundInit;
		
		
		//Storage array for the last n received (and transformed) PTAM 
		//positions and orientations. This array is used in the velocity estimate,
		int storageLength;						//The number of old frames to keep.
		int currIdx;							//The current location in the array.
		TooN::Vector<3, double> * ptamStorePos;     //Pointers to the storage arrays (position,euler angles, and time)
		TooN::Vector<3, double> * ptamStoreEuler;
		ros::Time * ptamStoreTime;
		
		//Flag to indicate if Vicon should be used.
		int viconOn;
};
#endif

//eof
