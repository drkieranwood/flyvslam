#include <flyvslam/ptam_data.h>
#include <r_apply_q.h>
#include <r_multi_q.h>
#include <r_check_q.h>
#include <r_q_to_e.h>
#include <r_e_to_q.h>


//========================
//========================
//Constructor.
ptam_data::ptam_data()
{
	currentPos   = TooN::makeVector(0.0,0.0,0.0);
	currentVel   = TooN::makeVector(0.0,0.0,0.0);
	currentRot   = TooN::makeVector(1.0,0.0,0.0,0.0);
	currentEuler = TooN::makeVector(0.0,0.0,0.0);
	currentYaw   = 0.0;

	initPtamPos = TooN::makeVector(0.0,0.0,0.0);
	initPtamRot = TooN::makeVector(1.0,0.0,0.0,0.0);
	setPtamInit = 0;

	initViconPos = TooN::makeVector(0.0,0.0,0.0);
	initViconRot = TooN::makeVector(1.0,0.0,0.0,0.0);
	setViconInit = 0;
	
	initCamerCorr = TooN::makeVector(0.0,0.0,0.0);
	cameraOffset  = TooN::makeVector(0.195,0.0,0.0);
	setCameraCorr = 0;

	ptamScale = 1.0;
	groundOrientation    = TooN::makeVector(1.0,0.0,0.0,0.0);
	groundOrientationInv = TooN::makeVector(1.0,0.0,0.0,0.0);
	groundPosition       = TooN::makeVector(0.0,0.0,0.0);
	setGroundInit = 0;
	
	//This is the number of points that should be used for the velocity estimate.
	//Setting 3 would use the average of two backwards differences.
	storageLength = 5;
	currIdx = 0;
	ptamStorePos   = new TooN::Vector<3, double>[storageLength];
	ptamStoreEuler = new TooN::Vector<3, double>[storageLength];
	ptamStoreTime  = new ros::Time[storageLength];
	
	//Set if the vicon pose correction should be used.
	viconOn = 1;
}


//========================
//========================
//Extract the current position and yaw from the ptam message and update the velocity. 
void ptam_data::update(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
	//Save the initial PTAM pose in the camera RF. Only do this once.
	if (setPtamInit==0)
	{
		initPtamPos = TooN::makeVector(msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z);
		initPtamRot = TooN::makeVector(msg->pose.pose.orientation.w,msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z);
		krot::r_check_q(initPtamRot);
		setPtamInit=1;
	}


	
	//Only transform to NED if the initialisation has been done.
	//This requires an initial PTAM frame and initial ground truth 
	//position and orientation to be set. The ground thruth may come 
	//from Vicon or can be set by another external source.
	if ( (setPtamInit==1) && (setGroundInit==1))
	{
		//========================	
		//TRANSFORM PTAM TO NED
		//========================

		//First extract the pose from the message
		TooN::Vector<3, double> workingPos = TooN::makeVector(msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z);	
		TooN::Vector<4, double> workingRot = TooN::makeVector(msg->pose.pose.orientation.w,msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z);
		krot::r_check_q(workingRot);

		//Invert the affine transform and apply scale.
		workingPos = (-1)*(krot::r_apply_q( (workingPos*ptamScale) ,workingRot));
		workingRot = krot::r_inv_q(workingRot);
		krot::r_check_q(workingRot);

		//Invert the initial PTAM pose at the current scale.
		TooN::Vector<3,double> initPtamPosInv = (-1)*(krot::r_apply_q( (initPtamPos*ptamScale) ,initPtamRot));
		TooN::Vector<4,double> initPtamRotInv = krot::r_inv_q(initPtamRot);

		//========================	
		//Position to NED
		//========================
		//Remove initial ptam position from measurement
		workingPos = workingPos - initPtamPosInv;
		//Rotate by initial ptam orientation
		workingPos = (-1)*(krot::r_apply_q(workingPos,initPtamRotInv));		
		//Swap axes order
		workingPos = TooN::makeVector(workingPos[2],workingPos[0],workingPos[1]);
		//Un-rotate by ground truth orientation.
		workingPos = (-1)*(krot::r_apply_q(workingPos, groundOrientationInv ));
		//Add initial ground truth
		workingPos = workingPos + groundPosition;

		//========================
		//Orientation to NED
		//========================
		//Find difference to initial orientation. qt = q0^-1 * qi
		//NOTE: used initPtamRot since it is the inverse of initPtamRotInv.
		workingRot = krot::r_multi_q(initPtamRot,workingRot);
		//Swap axes order
		workingRot = TooN::makeVector(workingRot[0],workingRot[3],workingRot[1],workingRot[2]);
		//Add initial ground truth orientation.
		//NOTE: groundOrientation can be provded by Vicon or by the averaging method.
		//if Vicon then the quaternion is applied 
		workingRot = krot::r_multi_q(groundOrientation,workingRot);
		krot::r_check_q(workingRot);

		//========================
		//Remove camera offset
		//========================	
		//Project [0.195,0,0]' into the world frame using the inverse of the current orientation
		TooN::Vector<4,double> tmp_invRot = krot::r_inv_q(workingRot);
		TooN::Vector<3,double> cameraCorr = krot::r_apply_q(cameraOffset,tmp_invRot);
		//Apply correction in world frame
		workingPos = workingPos - cameraCorr;
		
		
		//========================
		//Outputs
		//========================
		currentPos = workingPos;
		currentRot = workingRot;
		currentEuler = krot::r_q_to_e(currentRot);
		currentYaw = currentEuler[2];
		
			
		//========================
		//Find velocity
		//========================	
		//Store the pose into the current index location and the time-stamp.
		ptamStorePos[currIdx] = currentPos;
		ptamStoreEuler[currIdx] = currentEuler;
		ptamStoreTime[currIdx].sec  =  msg->header.stamp.sec;
		ptamStoreTime[currIdx].nsec =  msg->header.stamp.nsec;
		
		//Move through the data calculating the n-1 backwards differences.
		TooN::Vector<3, double> workingVelPos = TooN::makeVector(0.0,0.0,0.0);
		TooN::Vector<3, double> workingVelRot = TooN::makeVector(0.0,0.0,0.0);
		TooN::Vector<3, double> tmp_vel_pos;
		TooN::Vector<3, double> tmp_vel_rot;
		int oldestIdx = currIdx;
		for (int ii=0 ; ii<(storageLength-1) ; ii++)
		{
			//The next position in the array represents the oldest data.
			//If at the end then loop.
			oldestIdx = oldestIdx + 1;
			if (oldestIdx == storageLength)
			{
				oldestIdx = 0;
			}
			
			//The next next position is needed to form a difference. 
			//If at then end then loop.
			int newestIdx = oldestIdx + 1;
			if (newestIdx==storageLength)
			{
				newestIdx=0;
			}
			
			//Use the values to find a backwards difference velocity. Only if data is not identical.
			if(ptamStorePos[oldestIdx] != ptamStorePos[newestIdx])
			{
				double dt = (double)(ptamStoreTime[newestIdx]-ptamStoreTime[oldestIdx]).toSec();
				if(dt > 0.0)
				{
					TooN::Vector<3, double> temp_current = ptamStorePos[newestIdx];
					TooN::Vector<3, double> temp_previous = ptamStorePos[oldestIdx];
					tmp_vel_pos = TooN::makeVector((temp_current[0]-temp_previous[0])/dt,
					(temp_current[1]-temp_previous[1])/dt,
					(temp_current[2]-temp_previous[2])/dt);
					
					temp_current = ptamStoreEuler[newestIdx];
					temp_previous = ptamStoreEuler[oldestIdx];
					tmp_vel_rot = TooN::makeVector((temp_current[0]-temp_previous[0])/dt,
					(temp_current[1]-temp_previous[1])/dt,
					(temp_current[2]-temp_previous[2])/dt);
				}
			}
			else
			{
				//Set velocity to zero
				tmp_vel_pos = TooN::makeVector(0.0,0.0,0.0);
				tmp_vel_rot = TooN::makeVector(0.0,0.0,0.0);
			}
			
			//(ii+1) indicates how many velocities have been averaged so 
			//now add the new value and re-average.
			workingVelPos = ((workingVelPos*ii) + tmp_vel_pos)/(ii+1.0);
			workingVelRot = ((workingVelRot*ii) + tmp_vel_rot)/(ii+1.0);
			
		}

		//Set the final Velocity as the output.
		currentVel = workingVelPos;
		
		//Move along one storage. If at the end then loop.
		currIdx = currIdx +1;
		if(currIdx == storageLength)
		{
			currIdx = 0; 
		}
	}
}


//========================
//========================
//Set the intial Vicon position when PTAM was initialised. 
//Note this is the position of the camera, hence any body frame offset needs to be added onto the initPos.
void ptam_data::setInitVicon(TooN::Vector<3, double> initPos,TooN::Vector<4, double> initRot)
{
	//Only allow this to occur once.
	if (setViconInit==0)
	{
		initViconPos = initPos;
		initViconRot = initRot;
		initViconRotInv = krot::r_inv_q(initViconRot);
		
		//Check the quaternions are valid and normalise.
		krot::r_check_q(initViconRot);
		krot::r_check_q(initViconRotInv);
			
		setViconInit = 1;
		
		//If Vicon is used for the pose correction.
		if (viconOn)
		{
			this->setInitGround(initPos,initRot);
		}
	}
}


//========================
//========================
//Set the PTAM scale. It is initialised to 1.0.
void ptam_data::setPtamScale(double scaleTemp)
{
	ptamScale = scaleTemp;
}


//========================
//========================
//Set the pose correction. Ground thruth position and orientation.
//Note this is the position of the camera, hence any body frame offset needs to be added onto the tempPos.
void ptam_data::setInitGround(TooN::Vector<3, double> tempPos,TooN::Vector<4, double> tempRot)
{
	//Different to the Vicon init this can be called more than once.
	groundPosition = tempPos;
	groundOrientation = tempRot;
	groundOrientationInv = krot::r_inv_q(groundOrientation);
		
	//Check the quaternions are valid and normalise.
	krot::r_check_q(groundOrientation);
	krot::r_check_q(groundOrientationInv);
	
	setGroundInit = 1;
}


//========================
//========================
//Set the ground orientation correction. Can be called during a flight.
void ptam_data::setGroundOrientation(TooN::Vector<4, double> tempRot)
{
	groundOrientation = tempRot;
	groundOrientationInv = krot::r_inv_q(groundOrientation);
		
	//Check the quaternions are valid and normalise.
	krot::r_check_q(groundOrientation);
	krot::r_check_q(groundOrientationInv);
}


//eof
