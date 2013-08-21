#include <flyvslam/ptam_data.h>
#include <r_apply_q.h>
#include <r_multi_q.h>
#include <r_check_q.h>
#include <r_q_to_e.h>

//Constructor.
ptam_data::ptam_data()
{
	currentPos = TooN::makeVector(0.0,0.0,0.0);
	currentRot = TooN::makeVector(1.0,0.0,0.0,0.0);
	currentEuler = TooN::makeVector(0.0,0.0,0.0);
	currentYaw = 0.0;

	initPtamPos = TooN::makeVector(0.0,0.0,0.0);
	initPtamRot = TooN::makeVector(1.0,0.0,0.0,0.0);
	setPtamInit = 0;

	initViconPos = TooN::makeVector(0.0,0.0,0.0);
	initViconRot = TooN::makeVector(1.0,0.0,0.0,0.0);
	setViconInit = 0;

	ptamScale = 1.0;
}


//Extract the current position and yaw from the ptam message and update the velocity. 
void ptam_data::update(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
	//Save the initial PTAM pose in the camera RF. Only do this once.
	if (setPtamInit==0)
	{
		initPtamPos = TooN::makeVector(msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z);
		initPtamRot = TooN::makeVector(msg->pose.pose.orientation.w,msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z);
		setPtamInit = 1;

		//Check the quaternion is valid and normalise.
		krot::r_check_q(initPtamRot);
	}


	
	//Only transform to NED if the initialisation has been done.
	//Until the scale is set is assumes a scale of 1.
	if ( (setPtamInit==1) && (setViconInit==1))
	{
		/******************
		//TRANSFORM TO NED RF
		******************/

		//First extract to temp variables from the message
		TooN::Vector<3, double> workingPos = TooN::makeVector(msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z);	
		TooN::Vector<4, double> workingRot = TooN::makeVector(msg->pose.pose.orientation.w,msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z);

		//Check the incomming quaternion
		krot::r_check_q(workingRot);



		//Invert the affine transform and scale
		workingPos = (-1)*(krot::r_apply_q( (workingPos*ptamScale) ,workingRot));
		workingRot = krot::r_inv_q(workingRot);

	//Check the quaternion
	krot::r_check_q(workingRot);

		//Find some useful values
		TooN::Vector<3,double> initPtamPosInv = (-1)*(krot::r_apply_q( (initPtamPos*ptamScale) ,initPtamRot));
		TooN::Vector<4,double> initPtamRotInv = krot::r_inv_q(initPtamRot);

		/******************
		//Position
		******************/
		//Remove initial position from measurement
		
		workingPos = workingPos - initPtamPosInv;
		//Rotate by initial orientation
		workingPos = (-1)*(krot::r_apply_q(workingPos,initPtamRotInv));
		//Swap axes order
		workingPos = TooN::makeVector(workingPos[2],workingPos[0],workingPos[1]);
		//Rotate by initial vicon orientation
		workingPos = (-1)*(krot::r_apply_q(workingPos, initViconRotInv ));
		//Add initial vicon pos
		workingPos = workingPos + initViconPos;

		/******************
		//Orientation
		******************/
		//Find difference to initial orientation. qt = q0^-1 * qi
		//NOTE: used initPtamRot since it is the inverse of initPtamRotInv.
		workingRot = krot::r_multi_q(initPtamRot,workingRot);
		//Swap axes order
		workingRot = TooN::makeVector(workingRot[0],workingRot[3],workingRot[1],workingRot[2]);
		//Add initial Vicon
		workingRot = krot::r_multi_q(initViconRot,workingRot);
		//Check the outgoing quaternion
		krot::r_check_q(workingRot);

		/******************
		//Output
		******************/
		currentPos = workingPos;
		currentRot = workingRot;
		currentEuler = krot::r_q_to_e(currentRot);
		currentYaw = currentEuler[2];

	}
}

//Set the intial Vicon position when PTAM was initialised
void ptam_data::setInitVicon(TooN::Vector<3, double> initPos,TooN::Vector<4, double> initRot)
{
	//Only do this once
	if (setViconInit==0)
	{
		initViconPos = initPos;
		initViconRot = initRot;
		setViconInit = 1;
		initViconRotInv = krot::r_inv_q(initViconRot);

		//Check the quaternion is valid and normalise.
		krot::r_check_q(initViconRot);
		krot::r_check_q(initViconRotInv);
	}
}

//Set the PTAM scale
void ptam_data::setPtamScale(double scaleTemp)
{
	ptamScale = scaleTemp;
}



//eof
