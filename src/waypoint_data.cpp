#include <flyvslam/waypoint_data.h>
#include <iostream>
#include <fstream>
#include <string>

///Constructor doesn't do much. The readWaypointData() function is used for most of the object initialization
waypoint_data::waypoint_data()
{
	waypointCount=0;
	currentIdx=0;
	waypointLoops=0;
}

///Uses the current stored targetPos and targetLook (which have been lerp'd) to find the current target yaw angle. 
///Returns the yaw angle in radians as a double. Requires getTargetPos() has been called recently.
double waypoint_data::getTargetYaw()
{
	//targetYaw = atan(del_y,del_x)
	double targetYaw = atan2(targetLook[1]-targetPos[1],targetLook[0]-targetPos[0]);
	return targetYaw;
}


///Uses the current time, the start time and the current waypoint index to find the current target position and current target look position.
///Returns the current arget position as a toon vecor. Internally stores the current look position.
TooN::Vector<3, double> waypoint_data::getTargetPos(ros::Time timeNow)
{
	//Switch to next waypoint if the current waypoint transition time has been exceeded. 
	if(ros::Time::now().toSec() > (waypoint_time+waypointsTime[currentIdx]).toSec())
	{	
		//Set the transition time. Note: not based upon the time now but rather the time when it should have changed.
		waypoint_time = waypoint_time + waypointsTime[currentIdx];

		//Increment index and wrap if at the end
		currentIdx++;
		if(currentIdx == waypointCount)
		{
			waypointLoops++;
			currentIdx = 0;
		}
		ROS_INFO("Changed to waypoint: %i", (currentIdx+1));
	}


	//Lerp between current index and next index. Wrap to first waypoint if at the end.
	int nextIdx = currentIdx+1;
	if(nextIdx == waypointCount)
	{
		nextIdx = 0;
	}


	//Find the vector between the current waypoint and the next one.
	TooN::Vector<3, double> tmpvecPos  = waypointsPos[nextIdx]  - waypointsPos[currentIdx];
	TooN::Vector<3, double> tmpvecLook = waypointsLook[nextIdx] - waypointsLook[currentIdx];

	//Find the proportion of time elepsed since the last waypoint change as a ratio with the transition time.
	double lerpval = (double)((timeNow-waypoint_time).toSec()/waypointsTime[currentIdx].toSec());

	//Find the position the MAV should be right now and the point it should be looking at
	targetPos  = waypointsPos[currentIdx]  +(tmpvecPos *lerpval);
	targetLook = waypointsLook[currentIdx] +(tmpvecLook*lerpval);

	return targetPos;
}

bool waypoint_data::readWaypointData()
{
	//Create temporary storage for up to 100 waypoints.
	int tmp_waypoint_count = 0;
	TooN::Vector<3, double> tmp_waypoints_cog[100];
	TooN::Vector<3, double> tmp_waypoints_look[100];	
	double tmp_waypoints_time[100];
	
	//This assumes the node is being run from the same directory as the "waypoints.txt" file.
	std::string line;
	std::ifstream myfile("waypoints.txt");

	//Check if the file is open.	
	if (myfile.is_open())
	{
		ROS_INFO("File opened...");
		//Read waypoint info from file into tmp waypoints until end of file. Ignore lines starting with a #.
		while (myfile.good())
		{
			
			getline (myfile,line);
			if((line!="")&&(line.find("#")!=0))
			{
				if(myfile.good())
				{
					//Create a cstring to hold the entire line
					char * cstr;
					cstr = new char [line.size()+1];

					//Copy the line data into a c strng (char array)
    					strcpy (cstr, line.c_str());

					//Create a temp storage for the x,y,z pos x,y,z look and duration (7 in total)
    					double * temp_read_values;
					temp_read_values = new double[7];

					//Create a pointer to the end of the char string
					char * pEnd;
	
					//Read the string as a double until the first white space (set pEnd as the position in the string reached)
					temp_read_values[0] = (double)strtod(cstr,&pEnd);

					//Go from the last end point and keep reading doubles until all 7 have been read
					for(int ii=1 ; ii<7 ; ii++)
					{
						temp_read_values[ii] = (double)strtod(pEnd,&pEnd);
					}

					//Store into temporary vectors as values
					tmp_waypoints_cog[tmp_waypoint_count] =TooN::makeVector(temp_read_values[0],temp_read_values[1],temp_read_values[2]);
					tmp_waypoints_look[tmp_waypoint_count]=TooN::makeVector(temp_read_values[3],temp_read_values[4],temp_read_values[5]);
					tmp_waypoints_time[tmp_waypoint_count]=temp_read_values[6];

					//Increment the total waypoint count
					tmp_waypoint_count++;
				}
			}
		}

		//Set the total number of waypoints.
		waypointCount = tmp_waypoint_count;		

		//Create an array of position vecotrs and durations.
		waypointsPos  = new TooN::Vector<3, double>[waypointCount];
		waypointsLook = new TooN::Vector<3, double>[waypointCount];
		waypointsTime = new ros::Duration[waypointCount];

		//Fill in the values.
		for(int ii=0 ; ii<waypointCount ; ii++)
		{
			waypointsPos[ii]  = tmp_waypoints_cog[ii];
			waypointsLook[ii] = tmp_waypoints_look[ii];
			waypointsTime[ii] = ros::Duration(tmp_waypoints_time[ii]);
		}
		
		//Close the file for neatness.
		myfile.close();

		ROS_INFO("...read %i waypoints",waypointCount);
		return 1;
	}
	else
	{
		//If the file is not opened then use some default vakues
		ROS_INFO("WARNING: Could not find waypoints.txt in current dir. Using default values");
		
		waypointCount = 2;
		waypointsPos  = new TooN::Vector<3, double>[waypointCount];
		waypointsPos[0] = TooN::makeVector(0,0,-1);
		waypointsPos[1] = TooN::makeVector(1,0,-1);

		waypointsLook = new TooN::Vector<3, double>[waypointCount];
		waypointsLook[0] = TooN::makeVector(9999,0,-1);
		waypointsLook[1] = TooN::makeVector(9999,0,-1);

		waypointsTime = new ros::Duration[waypointCount];
		waypointsTime[0]=ros::Duration(5.0);
		waypointsTime[1]=ros::Duration(10.0);
		return -1;
	}
}
//eof
