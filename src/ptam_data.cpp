#include <flyvslam/ptam_data.h>
#include <r_apply_q.h>
#include <r_multi_q.h>
#include <r_check_q.h>
#include <r_q_to_e.h>

//Constructor.
ptam_data::ptam_data()
{
	currentPos = TooN::makeVector(0.0,0.0,0.0);
	currentVel = TooN::makeVector(0.0,0.0,0.0);
	currentRot = TooN::makeVector(1.0,0.0,0.0,0.0);
	currentEuler = TooN::makeVector(0.0,0.0,0.0);
	currentYaw = 0.0;

	initPtamPos = TooN::makeVector(0.0,0.0,0.0);
	initPtamRot = TooN::makeVector(1.0,0.0,0.0,0.0);
	setPtamInit = 0;

	initViconPos = TooN::makeVector(0.0,0.0,0.0);
	initViconRot = TooN::makeVector(1.0,0.0,0.0,0.0);
	setViconInit = 0;
	
	initCamerCorr = TooN::makeVector(0.0,0.0,0.0);
	cameraOffset = TooN::makeVector(0.195,0.0,0.0);
	setCameraCorr = 0;

	ptamScale = 1.0;
	
	ptamCount = 3;
	ptamIdx = 0;
	ptamsPos =  new TooN::Vector<3, double>[ptamCount];
	ptamsTime = new ros::Time[ptamCount];
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
	//Until the scale is set is assumes a scale of 1.0.
	if ( (setPtamInit==1) && (setViconInit==1))
	{
		//========================	
		//TRANSFORM PTAM TinitCamerCorrO NED
		//there are few calls to check the quaternion is valid but this
		//function also re-normalises the value for increased accuracy.
		//========================

		//First extract to temp variables from the message
		TooN::Vector<3, double> workingPos = TooN::makeVector(msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z);	
		TooN::Vector<4, double> workingRot = TooN::makeVector(msg->pose.pose.orientation.w,msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z);
			//Check the incomming quaternion
			krot::r_check_q(workingRot);

		//Invert the affine transform and apply scale
		workingPos = (-1)*(krot::r_apply_q( (workingPos*ptamScale) ,workingRot));
		workingRot = krot::r_inv_q(workingRot);
			//Check the quaternion again
			krot::r_check_q(workingRot);

		//Find the inverses of the initial PTAM with the current scale value.
		TooN::Vector<3,double> initPtamPosInv = (-1)*(krot::r_apply_q( (initPtamPos*ptamScale) ,initPtamRot));
		TooN::Vector<4,double> initPtamRotInv = krot::r_inv_q(initPtamRot);

		//========================	
		//Position to NED
		//========================
		//Remove initial position from measurement
		workingPos = workingPos - initPtamPosInv;
		//Rotate by initial orientation
		workingPos = (-1)*(krot::r_apply_q(workingPos,initPtamRotInv));		
		//Swap axes order
		workingPos = TooN::makeVector(workingPos[2],workingPos[0],workingPos[1]);
		//Un-rotate by initial Vicon orientation
		workingPos = (-1)*(krot::r_apply_q(workingPos, initViconRotInv ));
		//Add initial Vicon position
		workingPos = workingPos + initViconPos;

		//========================
		//Orientation to NED
		//========================
		//Find difference to initial orientation. qt = q0^-1 * qi
		//NOTE: used initPtamRot since it is the inverse of initPtamRotInv.
		workingRot = krot::r_multi_q(initPtamRot,workingRot);
		//Swap axes order
		workingRot = TooN::makeVector(workingRot[0],workingRot[3],workingRot[1],workingRot[2]);
		//Add initial Vicon rotation
		workingRot = krot::r_multi_q(initViconRot,workingRot);
			//Check the outgoing quaternion
			krot::r_check_q(workingRot);

		//========================
		//Remove camera offset
		//========================	
		//Project [0.195,0,0]' into the world frame using the inverse of the current orientation
		TooN::Vector<4,double> tmp_invRot = krot::r_inv_q(workingRot);
		TooN::Vector<3,double> cameraCorr = krot::r_apply_q(cameraOffset,tmp_invRot);
		//If the first time then store the correction
		if (setCameraCorr==0)
		{
				initCamerCorr = cameraCorr;
				setCameraCorr = 1;
		}
		//Apply correction in world frame
		workingPos = workingPos - (cameraCorr - initCamerCorr);
		
			
		//========================
		//Find velocity
		//========================	
		//Store the velocity into the  current index location
		ptamsPos[ptamIdx] = workingPos;
		ptamsTime[ptamIdx].sec  =  msg->header.stamp.sec;
		ptamsTime[ptamIdx].nsec =  msg->header.stamp.nsec;
		
		//The velocity is found between the current position and the nth previous position. 
		//Hence the next position in the storage vector will be n old.
		int nextIdx = ptamIdx+1;
		if(nextIdx  == ptamCount)
		{
			nextIdx = 0;
		}
		
		//Find the velocity using latest measurements (only if the data is not identical)
		if(ptamsPos[ptamIdx] != ptamsPos[nextIdx])
		{
			double dt = (double)(ptamsTime[ptamIdx]-ptamsTime[nextIdx]).toSec();
			if(dt > 0.0)
			{
				TooN::Vector<3, double> temp_current = ptamsPos[ptamIdx];
				TooN::Vector<3, double> temp_previous = ptamsPos[nextIdx];
				TooN::Vector<3, double> tmp_vel = TooN::makeVector((temp_current[0]-temp_previous[0])/dt,
				(temp_current[1]-temp_previous[1])/dt,
				(temp_current[2]-temp_previous[2])/dt);
			
				//Store the velcocity (this is what the control reads from)
				currentVel = tmp_vel;
			}
		}
		
		//Move along one stroage. If at the end then wrap.
		ptamIdx++;
		if(ptamIdx == ptamCount)
		{
			ptamIdx = 0; 
		}


		//========================
		//Output
		//========================
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



//Set the PTAM scale. It is initialised to 1.0.
void ptam_data::setPtamScale(double scaleTemp)
{
	ptamScale = scaleTemp;
}



//eof
