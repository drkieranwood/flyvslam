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
	TooN::Vector<3, double> current_cmd_vel = TooN::makeVector(0,0,0);

	//Maximum number of times to repeat the waypoints. Put this value high for long flights.
	int maxWaypointLoops = 10;

	//Get the waypoints from file, and set the start time.
	waypoint_data waypoint_info;
	waypoint_info.readWaypointData();
	
	//=========================
	//Publishers and subscibers
	
	
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
	
	//Debug
	//ros::Publisher  pub_debug = n.advertise<geometry_msgs::Twist>("/debugout", 1);

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
	while (ros::ok() && (waypoint_info.waypointLoops < maxWaypointLoops) )
	{	
		//Update the the current position reference input and yaw
		TooN::Vector<3, double> referencePos = waypoint_info.getTargetPos(ros::Time::now());
		double referenceYaw = waypoint_info.getTargetYaw();
	
		//Update the current position and yaw from Vicon
		TooN::Vector<3, double> currentPos = vicon_info.currentPos;
		double currentYaw = vicon_info.currentYaw;
		
		/*
		std::cout << std::setw(6) << (180/3.14159)*vicon_info.currentEuler[0] << " " << (180/3.14159)*vicon_info.currentEuler[1] << " " << (180/3.14159)*vicon_info.currentEuler[2] << std::endl;
		geometry_msgs::Twist debugout;
		debugout.linear.x = vicon_info.currentPos[0];
		debugout.linear.y = vicon_info.currentPos[1];
		debugout.linear.z = vicon_info.currentPos[2];
		debugout.angular.x = vicon_info.currentEuler[0];
		debugout.angular.y = vicon_info.currentEuler[1];
		debugout.angular.z = vicon_info.currentEuler[2];
		pub_debug.publish(debugout);
		*/
		
		/****************************************
		//At this point in the loop we have the current position and yaw of the MAV expressed in the NED reference 
		//frame and the desired position and yaw. Now the control code starts.
		****************************************/
/*	
		// deturmine yaw direction and speed
		float angular_vel_mag = angle_dis(vicon_angle, atan2(follow_point.y-vicon_pos.y,follow_point.x-vicon_pos.x));
		//limit yaw speed
		if(fabs(angular_vel_mag) > 0.1)
		{
			angular_vel_mag = 0.1;
		}
		float angular_vel = angular_vel_mag*angle_lerp_direction_face_point_from_pos_angle(vicon_pos,vicon_angle,follow_point);
		
			
		v3 diff_vec = hover_point-vicon_pos;
		diff_vec.z = 0;
		float diff_vec_mag = vector_mag(diff_vec);		
		
		v3 tmp_norm (hover_point.x-vicon_pos.x,hover_point.y-vicon_pos.y,0);
		normalise_vector(tmp_norm);
		v3 tmp_tang (-tmp_norm.y,tmp_norm.x,0);
		
		//deturmine vel command
		float norm_mag = std::max(-1.0f,(float)std::min(1.0f,0.3f*diff_vec_mag-0.3f*dot_product(vicon_vel,tmp_norm)));
		float tang_mag = std::max(-1.0f,(float)std::min(1.0f,-0.3f*dot_product(vicon_vel,tmp_tang)));
		v3 tmp_vel = tmp_norm*norm_mag+tmp_tang*tang_mag;
		
		
		// limit vel magnitude
		if(vector_mag(tmp_vel) > 1.0f)
		{
			normalise_vector(tmp_vel);
			tmp_vel = tmp_vel*1.0f;
		}
		
		//axis transform from vicon to drone (z remains the same but isnt used)
		v3 drone_axis_x (cos(vicon_angle), sin(vicon_angle),0); 
		v3 drone_axis_y (-sin(vicon_angle), cos(vicon_angle),0);
		v3 tmp_vel_drone (dot_product(tmp_vel,drone_axis_x),dot_product(tmp_vel,drone_axis_y),0);
*/	
		//Publish movement commands to drones ros topic.
		//This is only performed if new Vicon data has been received in the last 1 second.
		if(1.0f > (ros::Time::now()-(vicon_info.vicon_last_update_time)).toSec())
		{
			//Create a message and fill in the command values
			geometry_msgs::Twist cmd_vel;
			//cmd_vel.linear.x = tmp_vel_drone.x;
			//cmd_vel.linear.y = tmp_vel_drone.y;
			//cmd_vel.linear.z = tmp_vel_drone.z;
			//cmd_vel.angular.z = angular_vel;
			cmd_vel.linear.x = 0;
			cmd_vel.linear.y = 0;
			cmd_vel.linear.z = 0;
			cmd_vel.angular.z = 0;
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
	} 

	//After main flight loop need to land and shut down
	sleepone.sleep();
	ROS_INFO("LANDING");
	{
		std_msgs::Empty tempMsg;
		pub_takeoff.publish(tempMsg);
		ros::spinOnce();
		sleepone.sleep();
		pub_takeoff.publish(tempMsg);
		ros::spinOnce();
		sleepone.sleep();
		pub_takeoff.publish(tempMsg);
		ros::spinOnce();
		sleepone.sleep();
	}

	ROS_INFO("flyvslam::End flight script.");
  	return 0;
}
//eof
