#include <flyvslam/vicon_data.h>
#include <r_q_to_e.h>

//Constructor sets the number of previous frames to store.
vicon_data::vicon_data()
{
	viconCount=3;
	currentVel=TooN::makeVector(0,0,0);
	currentPos=TooN::makeVector(0,0,0);
	currentRot=TooN::makeVector(0,0,0,0);

	currentEuler=TooN::makeVector(0,0,0);

	currentYaw=0.0;
	viconIdx=0;

	viconsPos =  new TooN::Vector<3, double>[viconCount];
	viconsRot =  new TooN::Vector<4, double>[viconCount];
	viconsTime = new ros::Time[viconCount];
}


//Extract the current position and yaw from the Vicon message and update the velocity. 
void vicon_data::update(const geometry_msgs::TransformStamped::ConstPtr& msg)
{
	//Update the latest received time (to check for dropouts)
	vicon_last_update_time = ros::Time::now();
	
	//Extract the position vector
	TooN::Vector<3, double> received_vicon_pos = TooN::makeVector(msg->transform.translation.x,msg->transform.translation.y,msg->transform.translation.z);
	TooN::Vector<4, double> received_vicon_rot = TooN::makeVector(msg->transform.rotation.w,msg->transform.rotation.x,msg->transform.rotation.y,msg->transform.rotation.z);


	//================================================================================
	// CONVERT INTO NED RF
	//================================================================================
	TooN::Vector<3, double> tmp_received_vicon_pos = received_vicon_pos;
	TooN::Vector<4, double> tmp_received_vicon_rot = received_vicon_rot;
	received_vicon_pos[0]=tmp_received_vicon_pos[0];
	received_vicon_pos[1]=(-1)*tmp_received_vicon_pos[1];
	received_vicon_pos[2]=(-1)*tmp_received_vicon_pos[2];

	received_vicon_rot[0]=tmp_received_vicon_rot[0];
	received_vicon_rot[1]=tmp_received_vicon_rot[1];
	received_vicon_rot[2]=(-1)*tmp_received_vicon_rot[2];
	received_vicon_rot[3]=(-1)*tmp_received_vicon_rot[3];
	//================================================================================
	//================================================================================

	
	//Store the Vicon data, extract the time info from the message time stamp.
	currentPos      = received_vicon_pos;
	vicon_time.sec  = msg->header.stamp.sec; 
	vicon_time.nsec = msg->header.stamp.nsec;

	//Add the data to the storage vectors
	viconsPos[viconIdx] = received_vicon_pos;
	viconsRot[viconIdx] = received_vicon_rot;
	viconsTime[viconIdx].sec  = msg->header.stamp.sec; 
	viconsTime[viconIdx].nsec = msg->header.stamp.nsec; 
	
	//The velocity is found between the current position and the nth previous position. 
	//Hence the next position in the storage vector will be n old.
	int nextIdx = viconIdx+1;
	if(nextIdx  == viconCount)
	{
		nextIdx = 0;
	}
	
	//Find the velocity using latest measurements (only if the data is not identical)
	if(viconsPos[viconIdx] != viconsPos[nextIdx])
	{
		double dt = (double)(viconsTime[viconIdx]-viconsTime[nextIdx]).toSec();
		if(dt > 0.0)
		{
			TooN::Vector<3, double> temp_current = viconsPos[viconIdx];
			TooN::Vector<3, double> temp_previous = viconsPos[nextIdx];
			TooN::Vector<3, double> tmp_vel = TooN::makeVector((temp_current[0]-temp_previous[0])/dt,
			(temp_current[1]-temp_previous[1])/dt,
			(temp_current[2]-temp_previous[2])/dt);
			
			//Store the velcocity (this is what the control reads from)
			currentVel = tmp_vel;
		}
	}

	//Find the current orientation as Euler angles
	currentEuler = krot::r_q_to_e(received_vicon_rot);
	currentYaw = currentEuler[2];
	
	//Move along one stroage. If at the end then wrap.
	viconIdx++;
	if(viconIdx == viconCount)
	{
		viconIdx = 0; 
	}

}
//eof
