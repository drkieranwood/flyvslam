#include <flyvslam/ptam_data.h>
#include <r_apply_q.h>
#include <r_multi_q.h>
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
	initPtamPosInv = TooN::makeVector(0.0,0.0,0.0);
	initPtamRotInv = TooN::makeVector(1.0,0.0,0.0,0.0);
	setPtamInit = 0;
	setPtamInitInv = 0;

	initViconPos = TooN::makeVector(0.0,0.0,0.0);
	initViconRot = TooN::makeVector(1.0,0.0,0.0,0.0);
	setViconInit = 0;

	ptamScale = 1.0;
}


//Extract the current position and yaw from the ptam message and update the velocity. 
void ptam_data::update(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
	//Save the initial PTAM pose. Only do this once.
	if (setPtamInit==0)
	{
		initPtamPos = TooN::makeVector(msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z);
		initPtamRot = TooN::makeVector(msg->pose.pose.orientation.w,msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z);
		setPtamInit = 1;
	}

	//Only transform to NED if the initialisation has been done
	if ( (setPtamInit==1) && (setViconInit==1))
	{
		//Apply the transform from VSLAM to NED

		//First extract to temp variables
		TooN::Vector<3, double> workingPos = TooN::makeVector(msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z);	
		TooN::Vector<4, double> workingRot = TooN::makeVector(msg->pose.pose.orientation.w,msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z);

		//Invert the affine transform and scale
		workingPos = (-1)*(krot::r_apply_q(workingPos*ptamScale,workingRot));
		workingRot = krot::r_inv_q(workingRot);
if (0)
{


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
		workingRot = krot::r_multi_q(initPtamRot,workingRot);
		//Swap axes order
		workingRot = TooN::makeVector(workingRot[0],workingRot[3],workingRot[1],workingRot[2]);
		//Add initial Vicon
		workingRot = krot::r_multi_q(initViconRot,workingRot);

		/******************
		//Output
		******************/
		currentPos = workingPos;
		currentRot = currentRot;
		currentEuler = krot::r_q_to_e(currentRot);
		currentYaw = currentEuler[2];
}


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
	}
}

//Set the PTAM scale
void ptam_data::setPtamScale(double scaleTemp)
{
	ptamScale = scaleTemp;
	if (setPtamInitInv==0) 
	{
		initPtamPosInv = (-1)*(krot::r_apply_q(initPtamPos*ptamScale,initPtamRot));
		initPtamRotInv = krot::r_inv_q(initPtamRot);
		setPtamInitInv = 1;
	}
}



//eof
