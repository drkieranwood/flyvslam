// src/slam_flyer.cp
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "turtlesim/TeleportAbsolute.h"
#include "turtlesim/Pose.h"
#include "turtlesim/Velocity.h"
#include <math.h>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <string>

#include <flyvslam/flyvslam.h>
#include <flyvslam/waypoint_data.h>
#include <flyvslam/vicon_data.h>
#include <flyvslam/ptam_data.h>
#include <r_wrap_pi.h>
#define PI 3.14159265


//Main flight control function. This,
///1)sets up the PTAM Vicon and waypoints, 
///2)starts a flight on Vicon control, 
///3)calibrates the VLSAM data into the NED frame.
///4)transfers flight control to ptam measurements.
int main(int argc, char **argv)
{
	ROS_INFO("flyvslam::Start flight script.");
	//Setup ROS node (register with roscore etc.)
	ros::init(argc, argv, "flyvslam");
	ros::NodeHandle n;
	
	//Create object to store and handle incomming Vicon data and PTAM data
	vicon_data vicon_info;
	ptam_data ptam_info;

	//Vector to hold the latest output to the MAV
	//TooN::Vector<3, double> current_cmd_vel = TooN::makeVector(0,0,0);

	//Maximum number of times to repeat the waypoints. Put this value high for long flights.
	int maxWaypointLoops = 1;
	int landingNow = 0;

	//Get the waypoints from file, and set the start time.
	waypoint_data waypoint_info;
	waypoint_info.readWaypointData();

	//Some helpful variables so the full object doesn't have to called every time.	
	TooN::Vector<3, double> referencePos;
	double referenceYaw;
	TooN::Vector<3, double> currentPos;
	TooN::Vector<3, double> currentVel;
	double currentYaw;
	TooN::Vector<3, double> ptamPos;
	double ptamYaw;

	//Flag to break out of PTAM control and back to safety control.
	//If set to 0 then the original control on Vicon will takeover and continue the waypoints.
	int PTAM_OK = 0;

	//Init of PTAM. First the baseline is set which has an arbitrary scale. Then a motion is performed
	//to calculate the scale by comparing the magnitudes of the vectors from VSLAM and vicon.
	int ptamInit = 0;
	int scaleInit = 0;
	//Values to get the scale
	TooN::Vector<3, double> ptamPos_one;
	TooN::Vector<3, double> ptamPos_two;
	TooN::Vector<3, double> viconPos_one;
	TooN::Vector<3, double> viconPos_two;

	

	//=========================
	//Publishers and subscibers
	//=========================
	
	//Subscribe to ros topics and setup callback functions. Also setup output node to ARDone. 
	//The Vicon incomming queue is set to 1 so some data is always stored.
	ros::Subscriber sub_vicon_pose = n.subscribe("/vicon/Archie/Archie", 1, &vicon_data::update,&vicon_info);
	ros::Publisher  pub_cmd_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

	//Setup a publisher to control takeoff and landing
	ros::Publisher  pub_takeoff = n.advertise<std_msgs::Empty>("/ardrone/takeoff",1);
	ros::Publisher  pub_land    = n.advertise<std_msgs::Empty>("/ardrone/land",1);

	//Setup publisher to control PTAM initialisation. 
	ros::Publisher  pub_ptaminit = n.advertise<std_msgs::String>("/vslam/key_pressed",1);

	//Setup subscriber to get the PTAM pose information
	ros::Subscriber sub_ptam_pose = n.subscribe("/vslam/pose", 1, &ptam_data::update,&ptam_info);
	

	//Temp debug output publishers
	ros::Publisher  pub_vicon_ned = n.advertise<geometry_msgs::Twist>("/vicon_ned", 1);
	ros::Publisher  pub_vslam_ned = n.advertise<geometry_msgs::Twist>("/vslam_ned", 1);

	//=========================
	//Timings
	//=========================

	//The timing is controled by a ros::rate object. The argument is the desired loop rate in Hz.
	ros::Rate rateLimiter(100);

	//Send the takeoff command after 2 seconds
	ros::Duration sleepone = ros::Duration(1,0);
	sleepone.sleep();
	sleepone.sleep();
	ROS_INFO("TAKEOFF");
	{
		std_msgs::Empty tempMsg;
		pub_takeoff.publish(tempMsg);
	}	

	//Set the initial start time for the waypoint object
	waypoint_info.start_time = ros::Time::now();
	waypoint_info.waypoint_time = ros::Time::now();

	//Main loop (loop forever)
	while (ros::ok() && (waypoint_info.waypointLoops < maxWaypointLoops) && (landingNow==0))
	{	
		/****************************************
		//GET SENSOR DATA
		****************************************/
		//Update the the current position reference input and yaw
		referencePos = waypoint_info.getTargetPos(ros::Time::now());
		referenceYaw = waypoint_info.getTargetYaw();
	
		//Update the current position and yaw from Vicon
		currentPos = vicon_info.currentPos;
		currentVel = vicon_info.currentVel;
		currentYaw = vicon_info.currentYaw;

		//Update current ptam pos and yaw (will be zeros for the first part of the flight)
		ptamPos = ptam_info.currentPos;
		ptamYaw = ptam_info.currentYaw;


		//Temp debug
		geometry_msgs::Twist vicon_ned;
		vicon_ned.linear.x = currentPos[0];
		vicon_ned.linear.y = currentPos[1];
		vicon_ned.linear.z = currentPos[2];
		vicon_ned.angular.x = vicon_info.currentEuler[0];
		vicon_ned.angular.y = vicon_info.currentEuler[1];
		vicon_ned.angular.z = vicon_info.currentEuler[2];
		pub_vicon_ned.publish(vicon_ned);

		geometry_msgs::Twist vslam_ned;
		vslam_ned.linear.x = ptamPos[0];
		vslam_ned.linear.y = ptamPos[1];
		vslam_ned.linear.z = ptamPos[2];
		vslam_ned.angular.x = ptam_info.currentEuler[0];
		vslam_ned.angular.y = ptam_info.currentEuler[1];
		vslam_ned.angular.z = ptam_info.currentEuler[2];
		pub_vslam_ned.publish(vslam_ned);	


		/****************************************
		//PTAM STUFF
		****************************************/
		//Check if the waypoint for start init.
		if((waypoint_info.currentIdx==2) && (ptamInit==0))
		{
			//Send the ptam calib command
			ROS_INFO("PTAM Calib. Start Command Sent");
			std_msgs::String initCmd;
 			initCmd.data = "Space";
			pub_ptaminit.publish(initCmd);
			ptamInit = 1;
		}
		//Check if the waypoint for stop init.
		if((waypoint_info.currentIdx==4) && (ptamInit==1))
		{
			//Send the ptam calib command
			ROS_INFO("PTAM Calib. Stop Command Sent");
			std_msgs::String initCmd;
 			initCmd.data = "Space";
			pub_ptaminit.publish(initCmd);
			ptamInit = 2;
			//Set the init Vicon pose
			ptam_info.setInitVicon(vicon_info.currentPos,vicon_info.currentRot);
		}	
		if((waypoint_info.currentIdx==5) && (scaleInit==0))
		{
			ROS_INFO("PTAM Calib. Scale Point 1");
			ptamPos_one = ptamPos;
			viconPos_one = currentPos;
			scaleInit = 1;
		}
		if((waypoint_info.currentIdx==7) && (scaleInit==1))
		{
			ROS_INFO("PTAM Calib. Scale Point 2");
			ptamPos_two = ptamPos;
			viconPos_two = currentPos;
			scaleInit = 2;

			//Find magnitude of vectors
			double ptamDist  = TooN::norm(ptamPos_two - ptamPos_one);
			double viconDist = TooN::norm(viconPos_two - viconPos_one);

			ptam_info.setPtamScale(double(viconDist/ptamDist));

			//Activate PTAM feedback
			PTAM_OK = 1;
		}

		
		//If the ptam data is deemed to be ok then swap to using ptam for control input
		if (PTAM_OK == 1) 
		{
			//If within 0.2m in all axes
			if ( ((fabs(currentPos[0]-ptamPos[0])) < 0.2) && ((fabs(currentPos[1]-ptamPos[1])) < 0.2) && ((fabs(currentPos[2]-ptamPos[2])) < 0.2))
			{
				currentPos = ptamPos;
				currentYaw = ptamYaw;
			}
		}

		/****************************************
		//At this point in the loop we have the current position and yaw of the MAV expressed in the NED reference 
		//frame and the desired position and yaw. Now the control code starts.
		//CONTROLLER
		****************************************/

		//========================	
		//Rotational
		//======================== 

		//Determine yaw control.
		//--find yaw error
		//--put in the range (-PI:PI]
		//--limit to +-0.1

		//Find angular error between current and desired yaw
		double angErr = referenceYaw-currentYaw;
		krot::r_wrap_pi(angErr);
	
		//Limit yaw speed
		if(angErr > 0.1)
		{
			angErr = 0.1;
		}
		if(angErr < -0.1)
		{
			angErr = -0.1;
		}
	
	
		
		//========================	
		//Translational
		//========================
		//Find position error in three exes
		TooN::Vector<3, double> errVec = referencePos-currentPos;
		//Set z error to zero temporarily
		errVec[3] = 0;
		//Find the 2D error total magnitude
		double errMag = TooN::norm(errVec);
		//Find 2D error unit direction and tangent direction (rotate 90degs)
		TooN::Vector<3, double> errNorm = TooN::unit(errVec);	
		TooN::Vector<3, double> errTang = TooN::makeVector((-1)*errNorm[1],errNorm[0],double(0.0));
	
		//Find normal and tangential components needed to move MAV to target.
		//Limit to range [-1:1]
		//Note that TooN uses the dot product when multiplying two vectors
		double norm_mag = std::max( -1.0 , std::min( 1.0 ,0.5f*errMag -0.5f*(currentVel*errNorm) ) );
		double tang_mag = std::max( -1.0 , std::min( 1.0 ,            -0.5f*(currentVel*errTang) ) );
		TooN::Vector<3, double> tmp_vel = errNorm*norm_mag + errTang*tang_mag;
	
		// Limit vel magnitude to 1
		if(TooN::norm(tmp_vel) > 1.0f)
		{
			TooN::normalize(tmp_vel);
			tmp_vel = tmp_vel*1.0f;
		}


		//For Z only use the current Z velocity and Z error to form a simple PD control.
		double diff_z = referencePos[2]-currentPos[2];
		double tmp_Z_cmd = 1.0f*diff_z - 0.8*currentVel[2];
		//Limit to +-1
		if(tmp_Z_cmd > 1.0f)
		{
			tmp_Z_cmd = 1.0f;
		}
		if(tmp_Z_cmd < -1.0f)
		{
			tmp_Z_cmd = -1.0f;
		}


		/****************************************
		//TRANSFORM AND OUTPUT (publish)
		//Manual commands can be injected by overwriting the cmd_vel message values below. 
		****************************************/		

		//Axis transform from vicon to drone (z remains the same but isnt used)
		TooN::Vector<3, double> drone_axis_x  = TooN::makeVector(cos(currentYaw), sin(currentYaw),0); 
		TooN::Vector<3, double> drone_axis_y  = TooN::makeVector(-sin(currentYaw), cos(currentYaw),0);
		TooN::Vector<3, double> tmp_vel_drone = TooN::makeVector(tmp_vel*drone_axis_x,tmp_vel*drone_axis_y,0);



		//Publish movement commands to drones ros topic.
		//This is only performed if new Vicon data has been received in the last 1 second.
		if(1.0f > (ros::Time::now()-(vicon_info.vicon_last_update_time)).toSec())
		{
			//Create a message and fill in the command values
			geometry_msgs::Twist cmd_vel;
			cmd_vel.linear.x = tmp_vel_drone[0];
			cmd_vel.linear.y = (-1)*tmp_vel_drone[1];
			cmd_vel.linear.z = (-1)*tmp_Z_cmd;
			cmd_vel.angular.z = (-1)*angErr;
			pub_cmd_vel.publish(cmd_vel);	
		}
		else
		{
			//Else no new Vicon data is available so send zeros
			ROS_INFO("No new Vicon data or nan error. Sending [0,0,0,0]' vel_cmd");
			geometry_msgs::Twist cmd_vel;
			cmd_vel.linear.x = 0;
			cmd_vel.linear.y = 0;
			cmd_vel.linear.z = 0;
			cmd_vel.angular.z = 0;
			pub_cmd_vel.publish(cmd_vel);	
		}
		
		//Check for new ros messages
		ros::spinOnce();
		rateLimiter.sleep();
		ros::spinOnce();
		
		//If at the last waypoint then land on next loop
		if (waypoint_info.currentIdx == (waypoint_info.waypointCount-1))
		{
			landingNow = 1;
		}
	} 

	//After main flight loop need to land and shut down
	sleepone.sleep();
	ROS_INFO("LANDING");
	{
		std_msgs::Empty tempMsg;

		pub_land.publish(tempMsg);
		ros::spinOnce();
		sleepone.sleep();

		pub_land.publish(tempMsg);
		ros::spinOnce();
		sleepone.sleep();

		pub_land.publish(tempMsg);
		ros::spinOnce();
		sleepone.sleep();
	}


	ROS_INFO("flyvslam::End flight script.");
  	return 0;
}
//eof
