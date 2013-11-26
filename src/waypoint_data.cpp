#include <flyvslam/waypoint_data.h>
#include <iostream>
#include <fstream>
#include <string>
#include <cmath>


//Constructor doesn't do much. 
//The readWaypointData() function is used 
//for most of this class's initialization.
waypoint_data::waypoint_data()
{
	waypointCount=0;
	currentIdx=0;
	targetYaw=0.0;
	targetPos=TooN::makeVector(0.0,0.0,0.0);
	targetLook=TooN::makeVector(0.0,0.0,0.0);
	teleopPos=TooN::makeVector(0.0,0.0,0.0);
	teleopYaw=0.0;
}



//Uses the current stored targetPos and targetLook (which have been 
//lerp'd inside the getTagetPos() function) to find the current target 
//yaw angle. Returns the yaw angle in radians as a double. 
//Requires getTargetPos() has been called recently.
double waypoint_data::getTargetYaw(void)
{
	//Uses the trig. --> targetYaw = atan2(delta_y,delta_x). Also checks for singularity.
	double delta_x = (targetLook[0]-targetPos[0]);
	double delta_y = (targetLook[1]-targetPos[1]);
	if ( (fabs(delta_x) + fabs(delta_y)) < 0.000001)
	{
		ROS_INFO("flyvslam::Target yaw singulaity. Using previous yaw");
	}
	else
	{
		targetYaw = atan2( delta_y , delta_x );
	}
	return (targetYaw+teleopYaw);
}



//Uses the current time, the start time and the current waypoint index 
//to find the current target position and current target look position 
//using linear interpolation. Returns the current target position as a 
//TooN vecor. Internally stores the current look position. Requires that 
//at least one call is made to this function between waypoints (i.e. the 
//waypoint transition time cannot be smaller than the sampling interval)

//Smooth curves can be specified by quantising circular paths into a 
//series of nodes. Just make sure the time-between nodes is larger than
//the control sampling rates.
TooN::Vector<3, double> waypoint_data::getTargetPos(ros::Time timeNow)
{
	//Switch to next waypoint if the current waypoint transition time 
	//has been exceeded. 
	if( (ros::Time::now().toSec()) > ((waypoint_time+waypointsTime[currentIdx]).toSec()) )
	{	
		//Set the transition time. NOTE: it is not based upon the 
		//time now, but rather the time when it should have changed.
		waypoint_time = waypoint_time + waypointsTime[currentIdx];

		//Increment the index position and wrap if at the end.
		currentIdx++;
		if(currentIdx == waypointCount)
		{
			currentIdx = 0;
		}
		ROS_INFO("flyvslam::Changed to waypoint: %i", (currentIdx+1) );
	}

	//Lerp between current index and next index, so find the next index 
	//value (wrap to first waypoint if at the end)
	int nextIdx = currentIdx+1;
	if(nextIdx == waypointCount)
	{
		nextIdx = 0;
	}

	//Find the vector between the current waypoint and look-point 
	//and the next ones.
	TooN::Vector<3, double> tmpvecPos  = waypointsPos[nextIdx]  - waypointsPos[currentIdx];
	TooN::Vector<3, double> tmpvecLook = waypointsLook[nextIdx] - waypointsLook[currentIdx];

	//Find the proportion of time elepsed since the last waypoint change 
	//as a ratio with the transition time.
	double lerpval = (double)((timeNow-waypoint_time).toSec()/waypointsTime[currentIdx].toSec());

	//Find the position the MAV should be right now and the point 
	//it should be looking at
	targetPos  = waypointsPos[currentIdx]  +(tmpvecPos *lerpval);
	targetLook = waypointsLook[currentIdx] +(tmpvecLook*lerpval);

	return (targetPos+teleopPos);
}



//Read a list of waypoint data from a file named waypoints.txt into a storage array.
//If the read fails the use some default waypoints near the origin.
//There can be no more than 200 waypoints specified.
bool waypoint_data::readWaypointData(void)
{
	//Create temporary storage for up to 200 waypoints.
	int tmp_waypointCount = 0;
	TooN::Vector<3, double> tmp_waypointsPos[200];
	TooN::Vector<3, double> tmp_waypointsLook[200];	
	double tmp_waypointsTime[200];
	
	//This assumes the node is being run from the same directory 
	//as the "waypoints.txt" file.
	//Open the file for reading.
	std::string line;
	std::ifstream myfile("waypoints.txt");

	if (myfile.is_open())
	{
		ROS_INFO("flyvslam::File opened...");
		//Read waypoint info from file into tmp waypoints until 
		//end of file. Ignore lines starting with a # or $ or blank.
		while (myfile.good())
		{			
			getline (myfile,line);		
			if (line.find("$")==0)                
			{
				//if a special commmand line starts with a $
			}
			else if((line!="")&&(line.find("#")!=0))		
			{
				//else if not a comment (starts with a #) or blank
				if(myfile.good())
				{
					//Create a cstring to hold the entire line
					char * cstr;
					cstr = new char [line.size()+1];

					//Copy the line data into a c string (char array)
    					strcpy (cstr, line.c_str());

					//Create a temp storage for the x,y,z pos x,y,z 
					//look and duration (7 in total)
    					double * temp_read_values;
					temp_read_values = new double[7];

					//Create a pointer to a position within the string.
					char * pEnd;
	
					//Read the string as a double until the first white 
					//space. Then set pEnd as the position in the string reached.
					//Store the value in the first position of the array.
					temp_read_values[0] = (double)strtod(cstr,&pEnd);

					//Moving from the previous point in the string, 
					//read the remining 6 values into the array.
					for(int ii=1 ; ii<7 ; ii++)
					{
						temp_read_values[ii] = (double)strtod(pEnd,&pEnd);
					}

					//Split the values into appropreate position, look, 
					//and time vectors.
					tmp_waypointsPos[tmp_waypointCount] =TooN::makeVector(temp_read_values[0],temp_read_values[1],temp_read_values[2]);
					tmp_waypointsLook[tmp_waypointCount]=TooN::makeVector(temp_read_values[3],temp_read_values[4],temp_read_values[5]);
					tmp_waypointsTime[tmp_waypointCount]=temp_read_values[6];

					//Once a yawpoint is added, increment the total 
					//waypoint counter
					tmp_waypointCount++;
				}
			}
		} //This loop breaks when at the end of the file.

		
		//Set the total number of waypoints.
		waypointCount = tmp_waypointCount;		

		//Create an array of position vecotrs and durations of the appropreate size.
		waypointsPos  = new TooN::Vector<3, double>[waypointCount];
		waypointsLook = new TooN::Vector<3, double>[waypointCount];
		waypointsTime = new ros::Duration[waypointCount];

		//Copy accross the values
		for(int aa=0 ; aa<waypointCount ; aa++)
		{
			waypointsPos[aa]  = tmp_waypointsPos[aa];
			waypointsLook[aa] = tmp_waypointsLook[aa];
			waypointsTime[aa] = ros::Duration(tmp_waypointsTime[aa]);
		}
		
		//Close the file for neatness.
		myfile.close();

		ROS_INFO("flyvslam::...read %i waypoints",waypointCount);
		
		//Return 1 to indicate waypoints were read from file.
		return 1;
	}
	else
	{
		//If the file is not opened then use some default values
		ROS_INFO("flyvslam:: Could not find waypoints.txt in current dir. Using default values.");
		
		//Default path is forwards and back 1m in the x axis at 1m altitude.
		waypointCount = 2;
		waypointsPos  = new TooN::Vector<3, double>[waypointCount];
		waypointsPos[0] = TooN::makeVector(0,0,-1);
		waypointsPos[1] = TooN::makeVector(1,0,-1);

		waypointsLook = new TooN::Vector<3, double>[waypointCount];
		waypointsLook[0] = TooN::makeVector(9999,0,-1);
		waypointsLook[1] = TooN::makeVector(9999,0,-1);

		waypointsTime = new ros::Duration[waypointCount];
		waypointsTime[0]=ros::Duration(10.0);
		waypointsTime[1]=ros::Duration(15.0);
		
		//Return -1 to indicate the file was not successfully read.
		return -1;
	}
	
	

}

//Receive the keyboard inputs
void waypoint_data::updateFromKeys(const geometry_msgs::TwistConstPtr& msg)
{
	//Store the keyboard inputs into the waypoint storage
	teleopPos = TooN::makeVector(msg->linear.x,msg->linear.y,msg->linear.z);
	teleopYaw = msg->angular.z ;
}
//eof
