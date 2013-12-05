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
#include <TooN/TooN.h>

#include <flyvslam/flyvslam.h>
#include <flyvslam/waypoint_data.h>
#include <flyvslam/vicon_data.h>
#include <flyvslam/ptam_data.h>
#include <flyvslam/lqg_control.h>
#include <flyvslam/redord_lqg.h>
#include <r_wrap_pi.h>
#include <r_e_to_q.h>
#include <r_apply_q.h>
#define PI 3.14159265358979323846


//Function to activate waypoint playback
int waypointPlaybackActive = 0;
void waypointPlaybackCallback(const std_msgs::EmptyConstPtr& msg)
{
	waypointPlaybackActive=1;
}


//Main flight control function. This does,
//1)sets up the ptam, vicon, and waypoints objects,
//2)starts a flight on Vicon control, 
//3)calibrates the VLSAM data into the NED frame,
//4)transfers flight control to ptam measurements,
//
//Any variable labelled NED represents the data after some transformation into the global NED reference frame.
int main(int argc, char **argv)
{
	ROS_INFO("flyvslam::Start flight script.");
	//Setup ROS node (register with roscore etc.)
	ros::init(argc, argv, "flyvslam");
	ros::NodeHandle n;
	
	
	//=========================
	//Objects and variables
	//=========================
	
	//Flight options. These flag control which controllers, feedback, etc. should be used.
	//Ideally they would be options in a GUI or command line parameters.
	int ptamControl_on = 0;
	int manualPtamInit_on = 0;
	int lqgControl_on = 1;
	int waypointPlayback_on = 0;
	int vision_on = 1;
	
	
	//Create objects to store and handle vicon, ptam, and waypoint data.
	//These are the main input handlers that make the data available to the 
	//control later.
	vicon_data vicon_info;
	ptam_data ptam_info;
	waypoint_data waypoint_info;
	
	
	//Flags to control loops, flow, and initialisations.
	//Some of these indicate when tasks have been completed and others,
	//are tolerances to swap between controls emergency actions.
	int usePtamFeedback_on = 0;	//Flag to break out of PTAM control back to the original, very stable, Vicon safety controller. 1=PTAM control, 0=Vicon control.
	int landingNow = 0;			//Flag to indicate the last waypoint has been reached and landing should occur. Can be used by other code to induce a landing at any time.
	int ptamInit = 0;			//Flag to control the PTAM baseline initilisation. When complete this is set to 2.
	int scaleInit = 0;			//Flag to control the PTAM scale initilisation. When complete this is set to 2.
	int takeOffInit = 0;
	
	
	//Create a set of LQG controllers (in the H2 state-space form).
	ROS_INFO("flyvslam::Read control gains X");
	lqg_control * controlX = loadControlGainsFromFile("../controlgains/controlX");
	ROS_INFO("flyvslam::Read control gains Y");
	lqg_control * controlY = loadControlGainsFromFile("../controlgains/controlY");
	ROS_INFO("flyvslam::Read control gains Z");
	lqg_control * controlZ = loadControlGainsFromFile("../controlgains/controlZ");
	ROS_INFO("flyvslam::Read control gains W");
	lqg_control * controlW = loadControlGainsFromFile("../controlgains/controlW");
	//If any controllers failed then do not take-off
	if ((controlX==NULL) || (controlY==NULL) || (controlZ==NULL) || (controlW==NULL))
	{
		landingNow=1;
	}
	
	int iX = controlX->getNumInputs();
	int iY = controlY->getNumInputs();
	int iZ = controlZ->getNumInputs();
	int iW = controlW->getNumInputs();
	int oX = controlX->getNumOutputs();
	int oY = controlY->getNumOutputs();
	int oZ = controlZ->getNumOutputs();
	int oW = controlW->getNumOutputs();
	
	
	//Create a set of reduced order LQG controllers
	ROS_INFO("flyvslam::Read control gains X");
	redord_lqg * redordX = loadRedordLQG("../controlgains/redordX");
	ROS_INFO("flyvslam::Read control gains Y");
	redord_lqg * redordY = loadRedordLQG("../controlgains/redordX");
	ROS_INFO("flyvslam::Read control gains Z");
	redord_lqg * redordZ = loadRedordLQG("../controlgains/redordZ");
	ROS_INFO("flyvslam::Read control gains W");
	redord_lqg * redordW = loadRedordLQG("../controlgains/redordW");
	//If any controllers failed then do not take-off
	if ((redordX==NULL) || (redordY==NULL) || (redordZ==NULL) || (redordW==NULL))
	{
		landingNow=1;
	}
	redordX->setMaxCtrl(0.37);
	redordX->setMinCtrl(-0.37);
	redordY->setMaxCtrl(0.37);
	redordY->setMinCtrl(-0.37);
	redordZ->setMaxCtrl(1.0);
	redordZ->setMinCtrl(-1.0);
	redordW->setMaxCtrl(1.0);
	redordW->setMinCtrl(-1.0);

	int iXr = redordX->getNumInputs();
	int iYr = redordY->getNumInputs();
	int iZr = redordZ->getNumInputs();
	int iWr = redordW->getNumInputs();
	int oXr = redordX->getNumOutputs();
	int oYr = redordY->getNumOutputs();
	int oZr = redordZ->getNumOutputs();
	int oWr = redordW->getNumOutputs();


			/*
			//Test it worked
			std::cout << "START TEST" << std::endl;
			TooN::Matrix<TooN::Dynamic,TooN::Dynamic,double> tempA = redordX->getAd();
			std::cout << tempA(0,0) << " " << tempA(0,1) << " " << tempA(0,2) << std::endl;
			std::cout << "END TEST" << std::endl;
			* */
			
	
	
	//Some helpful variables. These just extract and store a direct copy of data available in 
	//the vicon, ptam, and waypoint objects, to make the later code look cleaner.
	TooN::Vector<3, double> referencePos;		//the current desired position
	double referenceYaw;						//the current desired yaw
	TooN::Vector<3, double> viconPos;			//the current actual position as reported by Vicon(NED)
	TooN::Vector<4, double> viconRot;
	TooN::Vector<3, double> viconVel;			//the current actual velocity as reported by Vicon(NED) (this is a basic backwards diff.)
	double viconYaw;							//the current actual yaw as reported by Vicon(NED)
	TooN::Vector<3, double> ptamPos;			//the current position as reported by PTAM(NED)
	TooN::Vector<4, double> ptamRot;
	TooN::Vector<3, double> ptamVel;			//the current velocity as reported by PTAM(NED) (this is a basic backwards diff.)
	double ptamYaw;								//the current yaw as reported by Vicon(NED)
	TooN::Vector<3, double> prevPos = TooN::makeVector(0.0,0.0,0.0);		//Storage for previous loop position. Allows checks for non-changing inputs.

	
	//Storage for instantaneous position sample values used to calculate the scale. 
	TooN::Vector<3, double> ptamPos_one;
	TooN::Vector<3, double> ptamPos_two;
	TooN::Vector<3, double> viconPos_one;
	TooN::Vector<3, double> viconPos_two;
	
	
	//Variables to find the average roll and pitch.
	double avgRoll  = 0.0;
	double avgPitch = 0.0;
	int avgCount = 0;
	//This is to check if the current ptam output has not changed since the last loop.
	TooN::Vector<3, double> ptamCheckPos = TooN::makeVector(0.0,0.0,0.0);

	
	//=========================
	//Publishers and subscibers
	//=========================
	
	//Setup subscriber to get the Vicon pose information.
	//The queue is set to 1 to make sure only the latest measurement is parsed.
	ros::Subscriber sub_vicon_pose = n.subscribe("/vicon/Archie/Archie", 1, &vicon_data::update,&vicon_info);
	
	//AR.Drone output command (needs the ardrone_autonomy node running to actually send the command to the MAV).
	ros::Publisher  pub_cmd_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel_del", 1);

	//Setup a publisher to control takeoff and landing.
	ros::Publisher  pub_takeoff = n.advertise<std_msgs::Empty>("/ardrone/takeoff",1);
	ros::Publisher  pub_land    = n.advertise<std_msgs::Empty>("/ardrone/land",1);

	//Setup publisher to control PTAM initialisation. 
	ros::Publisher  pub_ptaminit = n.advertise<std_msgs::String>("/vslam/key_pressed",1);

	//Setup subscriber to get the PTAM pose information.
	//The queue is set to 1 to make sure only the latest measurement is parsed.
	ros::Subscriber sub_ptam_pose = n.subscribe("/vslam/pose", 1, &ptam_data::update,&ptam_info);
	
	//Output the NED reference frame measurements, as x,y,z,phi,theta,psi. Used for quick plotting and debugging.
	ros::Publisher  pub_vicon_ned = n.advertise<geometry_msgs::Twist>("/vicon_ned", 1);
	ros::Publisher  pub_vslam_ned = n.advertise<geometry_msgs::Twist>("/vslam_ned", 1);
	ros::Publisher  pub_refer_ned = n.advertise<geometry_msgs::Twist>("/refer_ned", 1);
	
	ros::Publisher  pub_vicon_vel = n.advertise<geometry_msgs::Twist>("/vicon_vel", 1);
	ros::Publisher  pub_vslam_vel = n.advertise<geometry_msgs::Twist>("/vslam_vel", 1);
	
	//Subscribe to the topic that sets the keyboard inputted reference position.
	ros::Subscriber sub_teleopref = n.subscribe("/ref_pose", 1, &waypoint_data::updateFromKeys,&waypoint_info);

	//Subscribe to the waypoint playback topic
	ros::Subscriber sub_waypointPlayback = n.subscribe("/ref_playback", 1, waypointPlaybackCallback);


	//=========================
	//Waypoint playback
	//=========================
	//A waypoint will attempt to be added every minWaypointTime seconds. If 
	//either minWaypointDist or minWaypointYaw have not been exceeded since the last 
	//waypoint then don't add another.
	double minWaypointAddTime = 3.0;						//Min time between waypoint collections
	double minWaypointDist = 0.5;							//Min distance between waypoint collections
	double minWaypointYaw = PI/9.0;							//Min yaw between waypoint collections
	ros::Time waypointLastAdd = ros::Time::now();			//The time at which the last waypoint was added
	
	TooN::Vector<3, double> waypointPlaybackPos[1000];		//Storage for the positions and yaw and times
	double waypointPlaybackYaw[1000];
	ros::Time waypointPlaybackTime[1000];
	
	double waypointPlaybackTimeDiff = 3.0;					//Playback time between waypoints. This is the speed of motion.
	ros::Time waypointPlaybackChangeTime;					//Time when last waypoint was changed.

	int waypointCount = 0;									//Total waypoints collected
	int waypointPlaybackCount = 0;							//Total waypoints played back so far
	int firstWaypointPlayback = 0;							//Flag to indicate the first playback iteration.


	//=========================
	//Timings
	//=========================

	//The timing is controled by a ros::rate object. The argument is the desired loop rate in Hz. 
	//Note this must be faster than the waypoint transition times. A counter system is used to make
	//the control evaluate at a lower rate. The control will evaluate every ctrlCountMax iterations
	//of the main ROS loop.
	ros::Rate rateLimiter(100);
	int ctrlCount    = 0;
	int ctrlCountMax = 10;
	//Set to 10 for 10Hz, 5 for 20Hz, 20 for 5Hz etc...
	

	//Send the take-off command after a 2 second delay. To allow all ROS init. processes to complete.
	ros::Duration sleepone = ros::Duration(1,0);
	sleepone.sleep();
	sleepone.sleep();
	
	//If not using the average roll and pitch method then take-off now,
	//else take-off is after then manual init procedure.
	if (manualPtamInit_on==0)
	{
		ROS_INFO("flyvslam::TAKEOFF");
		{
			std_msgs::Empty tempMsg;
			pub_takeoff.publish(tempMsg);
		}		
		takeOffInit = 1;
	}
	
	//Get the waypoints from file, and set the flight start time
	waypoint_info.readWaypointData();
	waypoint_info.start_time = ros::Time::now();
	waypoint_info.waypoint_time = ros::Time::now();
	

	
	//Main loop. Loop until last waypoint, then land.
	//Landing can be initiated at any time by setting landingNow=1.
	while (ros::ok() && (landingNow==0))
	{	
		/**************************************************************
		//GET SENSOR DATA 
		//start each loop by harvesting the latest input data
		**************************************************************/
		//Update the the current position reference input and yaw reference.
		referencePos = waypoint_info.getTargetPos(ros::Time::now());
		referenceYaw = waypoint_info.getTargetYaw();
	
		//Update the current position, velocity, and yaw from Vicon.
		viconPos = vicon_info.currentPos;
		viconRot = vicon_info.currentRot;
		viconVel = vicon_info.currentVel;
		viconYaw = vicon_info.currentYaw;

		//Update the current position, velocity, and yaw from PTAM.
		//These will be zeros for the first part of the flight, then arbirarily scaled, 
		//then become NED once the scale has been set.
		ptamPos = ptam_info.currentPos;
		ptamRot = ptam_info.currentRot;
		ptamVel = ptam_info.currentVel;
		ptamYaw = ptam_info.currentYaw;
		
		
		/**************************************************************
		//WAYPOINT PLAYBACK
		//This section collects waypoints during the flight then overwrites 
		//reference with the reverse route when activated. It make use of the
		//data just collected and can overwrite referencePos and referenceYaw.
		**************************************************************/
		if (waypointPlayback_on==1)
		{
			
			//========================	
			//Collect
			//========================
			//If enough time has elapsed then consider adding a marker, and when not in playback mode.
			//The total waypoints in the storage is (waypointCount-1)
			if ( (( (ros::Time::now() - waypointLastAdd).toSec() ) > minWaypointAddTime) && (waypointPlaybackActive==0))
			{
				//If no waypoints then start the storage with current position.
				if (waypointCount==0)
				{
					waypointPlaybackPos[waypointCount]  = ptamPos;
					waypointPlaybackYaw[waypointCount]  = ptamYaw;
					waypointPlaybackTime[waypointCount] = ros::Time::now();
					waypointCount++;
					ROS_INFO("flyvslam::Playback waypoint Added");
				}
				else
				{
					//Check to see if the MAV has moved or yawed far enough.
					double distErrTempWp = TooN::norm( waypointPlaybackPos[waypointCount-1] - ptamPos);
					double yawErrTempWp = waypointPlaybackYaw[waypointCount-1] - ptamYaw;
					if ( (distErrTempWp>minWaypointDist) || (yawErrTempWp>minWaypointYaw) )
					{
						waypointPlaybackPos[waypointCount]  = ptamPos;
						waypointPlaybackYaw[waypointCount]  = ptamYaw;
						waypointPlaybackTime[waypointCount] = ros::Time::now();
						waypointLastAdd = ros::Time::now();
						waypointCount++;
						ROS_INFO("flyvslam::Playback waypoint Added");
					}
				}
			}
		
			//========================	
			//Playback
			//========================
			//When the waypoint playback starts it should work from (waypointCount-1) back to zero.
			if (waypointPlaybackActive == 1)
			{
				//If this is the first enter of this loop then set the time.
				if (firstWaypointPlayback==0)
				{
					firstWaypointPlayback=1;
					waypointPlaybackChangeTime = ros::Time::now();
					ROS_INFO("flyvslam::Playback started");
				}
				
				//Need to lerp between [waypointCount-1] and [waypointCount-2] over waypointPlaybackTime seconds
				//Check if the next waypoint should be selected
				if ( ((ros::Time::now()-waypointPlaybackChangeTime).toSec()) > waypointPlaybackTimeDiff)
				{
					//Swap to next waypoint
					waypointPlaybackCount++;
					waypointPlaybackChangeTime = ros::Time::now();
				}
				
				//Find current and next indexes
				int curIdx = (waypointCount-1) - waypointPlaybackCount;
				int nxtIdx = curIdx-1;
				if (nxtIdx <0)
				{
					//If at the end then hold second to last last values
					referencePos = waypointPlaybackPos[1];
					referenceYaw = waypointPlaybackYaw[1];
				}
				else
				{
					//Find vec between current and next
					TooN::Vector<3, double> tempWaypointDiffVec = waypointPlaybackPos[nxtIdx] - waypointPlaybackPos[curIdx];
					double tempWaypointDiffYaw = waypointPlaybackYaw[nxtIdx] - waypointPlaybackYaw[curIdx];
					//Find proportion of time between last chaneg and now as ration of waypointPlaybackTime
					double tempLerpVal = ((ros::Time::now() - waypointPlaybackChangeTime).toSec())/waypointPlaybackTimeDiff;
					//Lerp the new target pos
					TooN::Vector<3, double> tempReferencePos = waypointPlaybackPos[curIdx] + tempWaypointDiffVec*tempLerpVal;
					double  tempReferenceYaw = waypointPlaybackYaw[curIdx] + tempWaypointDiffYaw*tempLerpVal;
					//Write these values as the reference position and yaw
					referencePos = tempReferencePos;
					referenceYaw = tempReferenceYaw;
				}
			}
		}
		


		{
			//Output the reference, Vicon, and PTAM, data as messages for logging,
			geometry_msgs::Twist vicon_ned;
			vicon_ned.linear.x = viconPos[0];
			vicon_ned.linear.y = viconPos[1];
			vicon_ned.linear.z = viconPos[2];
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
			
			geometry_msgs::Twist refer_ned;
			refer_ned.linear.x = referencePos[0];
			refer_ned.linear.y = referencePos[1];
			refer_ned.linear.z = referencePos[2];
			refer_ned.angular.x = 0.0;
			refer_ned.angular.y = 0.0;
			refer_ned.angular.z = referenceYaw;
			pub_refer_ned.publish(refer_ned);
			
			//Optional first order velocities
			if (0)
			{
				geometry_msgs::Twist vicon_vel;
				vicon_vel.linear.x = viconVel[0];
				vicon_vel.linear.y = viconVel[1];
				vicon_vel.linear.z = viconVel[2];
				vicon_vel.angular.x = 0.0;
				vicon_vel.angular.y = 0.0;
				vicon_vel.angular.z = 0.0;
				pub_vicon_vel.publish(vicon_vel);
				
				geometry_msgs::Twist vslam_vel;
				vslam_vel.linear.x = ptamVel[0];
				vslam_vel.linear.y = ptamVel[1];
				vslam_vel.linear.z = ptamVel[2];
				vslam_vel.angular.x = 0.0;
				vslam_vel.angular.y = 0.0;
				vslam_vel.angular.z = 0.0;
				pub_vslam_vel.publish(vslam_vel);
			}
			
		}


		/**************************************************************
		//PTAM initialisation motions
		//at the start of the flight two motions need perforing,
		//1) move to set a baseline for PTAM
		//2) move to find a scale for PTAM
		**************************************************************/
		if (vision_on)
		{
			//If at the third waypoint (idx==2) then start the PTAM baseline process.
			if((waypoint_info.currentIdx==2) && (ptamInit==0))
			{
				ROS_INFO("flyvslam::PTAM Baseline start");
				std_msgs::String initCmd;
				initCmd.data = "Space";
				pub_ptaminit.publish(initCmd);
				ptamInit = 1;
			}
			//If at the fith waypoint (idx==4) then stop the PTAM baseline process.
			if((waypoint_info.currentIdx==4) && (ptamInit==1))
			{
				ROS_INFO("flyvslam::PTAM Baseline stop");
				std_msgs::String initCmd;
				initCmd.data = "Space";
				pub_ptaminit.publish(initCmd);
				
				//Set the initial Vicon pose for the PTAM to NED transformation.
				ptam_info.setInitVicon(vicon_info.currentPos,vicon_info.currentRot);
				
				if (manualPtamInit_on==1)
				{
					//Set a manual pose correction. The orientation correction is set to do nothing at the moment. 
					//Hence the orientation output will be in the camera frame.
					//This overwrites the one set by the setinitVicon() above.
					TooN::Vector<3,double> initPosTemp = TooN::makeVector(1.0,-0.195,-1.0);
					TooN::Vector<4,double> initRotTemp = TooN::makeVector(1.0,0.0,0.0,0.0);
					ptam_info.setInitGround(initPosTemp,initRotTemp);
				}
				ptamInit = 2;
			}	
			//If at the sixth waypoint (idx==5) then start the scaling movement.
			if((waypoint_info.currentIdx==5) && (scaleInit==0))
			{
				ROS_INFO("flyvslam::PTAM Scale start");
				ptamPos_one = ptamPos;
				viconPos_one = viconPos;
				scaleInit = 1;
			}
			//If at the eighth waypoint (idx==7) then stop the scaling movement.
			if((waypoint_info.currentIdx==7) && (scaleInit==1))
			{
				ROS_INFO("flyvslam::PTAM Scale stop");
				ptamPos_two = ptamPos;
				viconPos_two = viconPos;
				//Find magnitude of vectors from Vicon and PTAM, to calculate and set the scale for the PTAM to NED transformation.
				double ptamDist  = TooN::norm(ptamPos_two - ptamPos_one);
				double viconDist = TooN::norm(viconPos_two - viconPos_one);
				
				//If using the no Vicon averaging method overwrtie the scale 
				//using the dead reckoning estimate.
				if (manualPtamInit_on==1)
				{
					ptam_info.setPtamScale(double(1.0/ptamDist));
				}
				else
				{
					ptam_info.setPtamScale(double(viconDist/ptamDist));
				}
				scaleInit = 2;
			}
			//If at the ninth waypoint (idx==8) then takeoff.
			//Only for the non-vicon initialisation.
			if((waypoint_info.currentIdx==8) && (takeOffInit==0))
			{
				//If using the averaging method then this is the time to takeoff.
				if (manualPtamInit_on==1)
				{
					ROS_INFO("flyvslam::TAKEOFF");
					{
						std_msgs::Empty tempMsg;
						pub_takeoff.publish(tempMsg);
					}
				}
				takeOffInit = 1;
			}

		
			//Once the ptam init has been completed create an
			//average roll and pitch output of PTAM. Turn this into a 
			//quaternion and send to the ptam correction.
			if (ptamInit==2 && avgCount<51 && (manualPtamInit_on==1))
			{
				//Only perform if PTAM has output a new measurement.
				if ( ptamCheckPos != ptamPos)
				{
					//Extract the roll and pitch from ptam
					double tmpRoll  = ptam_info.currentEuler[0];
					double tmpPitch = ptam_info.currentEuler[1];
					
					avgRoll   = ((avgRoll*avgCount)  + tmpRoll) /(avgCount+1.0);
					avgPitch  = ((avgPitch*avgCount) + tmpPitch)/(avgCount+1.0);
					avgCount = avgCount + 1;
					ROS_INFO("flyvslam::avgRoll=%6.5f, avgPitch:%6.5f",avgRoll,avgPitch);

					if (avgCount == 50)
					{
						TooN::Vector<3,double> tempEuler = TooN::makeVector( avgRoll,avgPitch,-1.5708 );
						TooN::Vector<4,double> tempQuat = krot::r_e_to_q( tempEuler );
						//Only set the correction after the average has been created.
						ROS_INFO("flyvslam::initAvgRot: %4.2f,%4.2f,%4.2f,%4.2f",tempQuat[0],tempQuat[1],tempQuat[2],tempQuat[3]);
						ptam_info.setGroundOrientation(tempQuat);
					}
									
					ptamCheckPos = ptamPos;
				}
			}
		}
		
		/**************************************************************
		//CONTROL SELECTION
		//check if the PTAM data is near the Vicon data, if so then use the PTAM
		//based controller, else break into the emergency controller.
		**************************************************************/
		//PTAM can only be used as feedback once it is initialised with 
		//scale and oreintation.
		//Also check the user flag (ptamControl_on) to see if it is required at all.
		if (ptamInit==2 && scaleInit==2 && (ptamControl_on))
		{
			if (usePtamFeedback_on!=1)
			{
					ROS_INFO("flyvslam::PTAM Control activated");
					usePtamFeedback_on = 1;
			}	
		}

		//If PTAM is to be used as the feedback then simply over-write the Vicon
		//positions etc with the PTAM data. All control calculations use viconPos, viconRot etc
		//but the actual values might be from PTAM as set here.
		//This is a little hacky but works.
		if (usePtamFeedback_on==1)
		{
			viconPos = ptamPos;
			viconVel = ptamVel;
			viconYaw = ptamYaw;
			viconRot = ptamRot;
		}
	
	
	
		/**************************************************************
		//PID CONTROL
		//pid control just uses the position and velocity estimates from 
		//the inputs to implement a simple control.
		**************************************************************/
		//If not using the LQG then use a PID. This uses the full rate of
		//feedback available.
		if (!lqgControl_on)
		{
			//========================	
			//Rotational
			//======================== 
			//Find angular error between current and desired yaw. Wrap to the range (-PI:PI].
			//This is a simple proportional controller but it works very well.
			double angErr = referenceYaw-viconYaw;
			krot::r_wrap_pi(angErr);
		
			//Limit yaw speed to +-0.1
			if(angErr > (0.1))  { angErr =  0.1; }
			if(angErr < (-0.1)) { angErr = -0.1; }
		
			//========================	
			//Translational
			//========================
			//Find position error in three exes
			TooN::Vector<3, double> errVec = referencePos-viconPos;
			//Set z error to zero temporarily
			errVec[2] = 0;
			//Find the 2D error total magnitude
			double errMag = TooN::norm(errVec);
			//Find 2D error unit direction and tangent direction (rotate 90degs)
			TooN::Vector<3, double> errNorm = TooN::unit(errVec);	
			TooN::Vector<3, double> errTang = TooN::makeVector((-1)*errNorm[1],errNorm[0],double(0.0));
		
			//Find normal and tangential components needed to move MAV to target.
			//Limit to range [-1:1]
			//NOTE: that TooN performs the dot product when multiplying two vectors.
			//Vicon P=0.5, D=0.5
			//PTAM  P=0.5, D=0.1
			double propGainXY;
			double diffGainXY;
			if (usePtamFeedback_on==1)
			{
				propGainXY = 0.3;
				diffGainXY = 0.1; 
			}
			else
			{
				propGainXY = 0.5;
				diffGainXY = 0.5; 
			}
			double norm_mag = std::max( -1.0 , std::min( 1.0 ,propGainXY*errMag -diffGainXY*(viconVel*errNorm) ) );
			double tang_mag = std::max( -1.0 , std::min( 1.0 ,                  -diffGainXY*(viconVel*errTang) ) );
			TooN::Vector<3, double> tmp_vel = errNorm*norm_mag + errTang*tang_mag;

			// Limit tmp_vel magnitude to 1
			if(TooN::norm(tmp_vel) > 1.0)
			{
				TooN::normalize(tmp_vel);
				tmp_vel = tmp_vel*1.0;
			}
			
			//========================	
			//Z axis
			//========================
			//For Z only use the current Z velocity and Z error to form a simple PD control.
			//Vicon P=1.0, D=0.8
			//PTAM  P=0.8, D=0.3
			double propGainZ;
			double diffGainZ;
			if (usePtamFeedback_on==1)
			{
				propGainZ = 0.8;
				diffGainZ = 0.3; 
			}
			else
			{
				propGainZ = 1.0;
				diffGainZ = 0.8; 
			}
			double diff_z = referencePos[2]-viconPos[2];
			double tmp_Z_cmd = propGainZ*diff_z - diffGainZ*viconVel[2];
			//Limit to +-1
			if(tmp_Z_cmd > (1.0) )  { tmp_Z_cmd = 1.0; }
			if(tmp_Z_cmd < (-1.0)) { tmp_Z_cmd = -1.0; }


			//========================	
			//Transform
			//========================		
			//Axis transform from Vicon to drone (z remains the same but is not used)
			TooN::Vector<3, double> drone_axis_x  = TooN::makeVector(cos(viconYaw), sin(viconYaw),0); 
			TooN::Vector<3, double> drone_axis_y  = TooN::makeVector(-sin(viconYaw), cos(viconYaw),0);
			TooN::Vector<3, double> tmp_vel_drone = TooN::makeVector(tmp_vel*drone_axis_x,tmp_vel*drone_axis_y,0);
			
			//========================	
			//Output
			//========================	
				
			//When usign Vicon check to see if the MAV is out of bounds by 
			//checking the last Vicon update time.
			if (usePtamFeedback_on==0)
			{			
				if( (1.0) > ((ros::Time::now()-(vicon_info.vicon_last_update_time)).toSec()) )
				{
					//If using Vicon feedback only publish if Vicon has new data.	
					geometry_msgs::Twist cmd_vel;
					cmd_vel.linear.x = tmp_vel_drone[0];
					cmd_vel.linear.y = (-1)*tmp_vel_drone[1];		//Axis negated to make NED
					cmd_vel.linear.z = (-1)*tmp_Z_cmd;				//Axis negated to make NED
					cmd_vel.angular.z = (-1)*angErr;				//Axis negated to make NED
					//Setting angular.x =1 ensures MAV never enters hover mode using the on-board controller.
				    cmd_vel.angular.x = 1;    
					pub_cmd_vel.publish(cmd_vel);	
				}
				else
				{
					//Else no new Vicon data is available so send zeros to activate hover mode.
					ROS_INFO("flyvslam::No new Vicon data or nan error. Sending [0,0,0,0]' vel_cmd");
					geometry_msgs::Twist cmd_vel;
					cmd_vel.linear.x = 0;
					cmd_vel.linear.y = 0;
					cmd_vel.linear.z = 0;
					cmd_vel.angular.x = 0;
					cmd_vel.angular.y = 0;
					cmd_vel.angular.z = 0;
					//Sending all zeros activates the hover mode.
					pub_cmd_vel.publish(cmd_vel);	
					

				}
			}
			else
			{
				//When using PTAM feedback it doesn't matter if the MAV is outside Vicon
				//so always publish the command.
				geometry_msgs::Twist cmd_vel;
				cmd_vel.linear.x = tmp_vel_drone[0];
				cmd_vel.linear.y = (-1)*tmp_vel_drone[1];		//Axis negated to make NED
				cmd_vel.linear.z = (-1)*tmp_Z_cmd;				//Axis negated to make NED
				cmd_vel.angular.z = (-1)*angErr;				//Axis negated to make NED
				//Setting angular.x =1 ensures MAV never enters hover mode using the on-board controller.
				cmd_vel.angular.x = 1;                          
				pub_cmd_vel.publish(cmd_vel);		
			}
		}
		
		
		/**************************************************************
		//LQG CONTROL
		//a LQG/H2 control using a single state-space implementation for each axis.
		**************************************************************/
		//This is the state-space LQG controller. 
		//Only evaluate if the LQG has been activated.
		if (lqgControl_on==1)
		{
			//There are two different criteria for performing a control update.
			//It is important to make usre the LQG is evaluated at the correct rate.
			int runUpdate = 0;
			if (usePtamFeedback_on==0)
			{
				//If using Vicon then update once per ctrlCountMax loops.
				if (ctrlCount==0)
				{
					runUpdate=1;
				}
				ctrlCount++;
				if (ctrlCount==ctrlCountMax)
				{
					ctrlCount=0;
				}
			}
			else
			{
				//If using PTAM feedback then only update when the PTAM position has changed.
				//The PTAM rate is limited externally by the image stream.
				if ((viconPos[0] == prevPos[0]) && (viconPos[1] == prevPos[1]) && (viconPos[2] == prevPos[2]) )
				{
						//Do nothing
				}
				else 
				{
					runUpdate=1;
				}
			}
			
			
			//Update the state-space LQG when required.
			if (runUpdate == 1)
			{
				/**************************************************************
				//LQG/H2 Control Version 1 - full auto-coded output from Matlab
				//this uses the LQG/H2 optimal controller as a set of state-space 
				//matrices which are updated once per-loop.
				**************************************************************/
				
				//===================
				//Need to find the body frame errors in x,y,z,w
				//===================
				//error = reference - current
				TooN::Vector<3,double> nedPosErr = referencePos-viconPos;
				double nedYawErr = referenceYaw-viconYaw;
				krot::r_wrap_pi(nedYawErr);   //wrap yaw error to (-PI:PI] so the MAV move the most direct direction
				
				//Rotate NED errors into the body frame.
				TooN::Vector<3,double> mavPosErr = krot::r_apply_q(nedPosErr,viconRot);
				
				//At this point the inputs to the state-space lqg/h2 controllers have been found.
				//[mavPosErr[0] mavPosErr[1] mavPosErr[2] nedYawErr]
				
				//===================
				//Update the controllers.
				//===================
				TooN::Vector<TooN::Dynamic,double> tempOutputX(oX);
				TooN::Vector<TooN::Dynamic,double> tempOutputY(oY);
				TooN::Vector<TooN::Dynamic,double> tempOutputZ(oZ);
				TooN::Vector<TooN::Dynamic,double> tempOutputW(oW);
				{
					TooN::Vector<TooN::Dynamic,double> tempInput(iX);
					tempInput = TooN::makeVector(mavPosErr[0]);
					tempOutputX = controlX->update(tempInput);
				}
				{
					TooN::Vector<TooN::Dynamic,double> tempInput(iY);
					tempInput = TooN::makeVector(mavPosErr[1]);
					tempOutputY = controlY->update(tempInput);
				}
				{
					TooN::Vector<TooN::Dynamic,double> tempInput(iZ);
					//Possibly replace mavPosErr[2] with nedPosErr[2] to stop some coupling between axes.
					tempInput = TooN::makeVector(nedPosErr[2]);
					tempOutputZ = controlZ->update(tempInput);
				}
				{
					TooN::Vector<TooN::Dynamic,double> tempInput(iW);
					tempInput = TooN::makeVector(nedYawErr);
					tempOutputW = controlW->update(tempInput);
				}
			
			
			/*
				//===================
				//Create command
				//===================
				//Fill in the control values to be sent to the MAV.
				geometry_msgs::Twist cmd_vel;
				cmd_vel.linear.x  = (-1.0)*tempOutputX[0];
				cmd_vel.linear.y  = (1.0)*tempOutputY[0];
				cmd_vel.linear.z  = (1.0)*tempOutputZ[0];
				cmd_vel.angular.z = (1.0)*tempOutputW[0];
				//Setting angular.x =1 ensures MAV never enters hover mode using the on-board controller.
				cmd_vel.angular.x = 1;  
				//Send to MAV
				pub_cmd_vel.publish(cmd_vel);	
			*/
			
			
			

				/**************************************************************
				//LQG/H2 Control Version 2 - reduced-order manual implementation
				//this uses the LQG/H2 optimal controller as a set of state-space 
				//matrices which are updated once per-loop.
				**************************************************************/
				//Find current BF position
				TooN::Vector<3,double> nedPos = krot::r_apply_q(viconPos,viconRot);
				
				//Find current BF demand
				TooN::Vector<3,double> nedRef = krot::r_apply_q(referencePos,viconRot);
				
				//The Y (2nd) element of these two vectors should be sent to the controller .
				//as current position and desired position.
				
			
				//Extract the x desired position into an augmented vector.
				//This is set to always be zero. (i.e. the controller works to make 
				//zero body frame error at all times for all states)
				int tempSize =0;
				tempSize = (redordX->getNumStates()) + (redordX->getNumDel()) + 1;
				TooN::Vector<TooN::Dynamic,double> augDesX(tempSize);
				augDesX = TooN::Zeros;
				tempSize = (redordY->getNumStates()) + (redordY->getNumDel()) + 1;
				TooN::Vector<TooN::Dynamic,double> augDesY(tempSize);
				augDesY = TooN::Zeros;
				tempSize = (redordZ->getNumStates()) + (redordZ->getNumDel()) + 1;
				TooN::Vector<TooN::Dynamic,double> augDesZ(tempSize);
				augDesZ = TooN::Zeros;
				tempSize = (redordW->getNumStates()) + (redordW->getNumDel()) + 1;
				TooN::Vector<TooN::Dynamic,double> augDesW(tempSize);
				augDesW = TooN::Zeros;
				
				//Change the 3rd value of Y to be the desired set point
				//augDesY[3] = nedRef[1];
				
				
				//===================
				//Update the controllers.
				//===================
				
				TooN::Vector<TooN::Dynamic,double> tempOutputXt(oXr);
				TooN::Vector<TooN::Dynamic,double> tempOutputYt(oYr);
				TooN::Vector<TooN::Dynamic,double> tempOutputZt(oZr);
				TooN::Vector<TooN::Dynamic,double> tempOutputWt(oWr);
				
				{
					TooN::Vector<TooN::Dynamic,double> tempInput(iXr);
					tempInput = TooN::makeVector(mavPosErr[0]);
					tempOutputXt = redordX->update(tempInput,augDesX);
				}
				
				{
					TooN::Vector<TooN::Dynamic,double> tempInput(iYr);
					//This uses the actual position expressed in the BF. Not the error.
					tempInput = TooN::makeVector(mavPosErr[1]);
					tempOutputYt = redordY->update(tempInput,augDesY);
				}
				{
					TooN::Vector<TooN::Dynamic,double> tempInput(iZr);
					//Possibly replace mavPosErr[2] with nedPosErr[2] to stop some coupling between axes.
					tempInput = TooN::makeVector(nedPosErr[2]);
					tempOutputZt = redordZ->update(tempInput,augDesZ);
				}
				{
					TooN::Vector<TooN::Dynamic,double> tempInput(iWr);
					tempInput = TooN::makeVector(nedYawErr);
					tempOutputWt = redordW->update(tempInput,augDesW);
				}
				
			
			
				//===================
				//Create command
				//===================
				//Fill in the control values to be sent to the MAV.
				geometry_msgs::Twist cmd_vel;
				cmd_vel.linear.x  = (-1.0)*tempOutputXt[0];
				cmd_vel.linear.y  = (1.0)*tempOutputYt[0];
				cmd_vel.linear.z  = (1.0)*tempOutputZt[0];
				cmd_vel.angular.z = (1.0)*tempOutputWt[0];
				//Setting angular.x =1 ensures MAV never enters hover mode using the on-board controller.
				cmd_vel.angular.x = 1;  
				//Send to MAV
				pub_cmd_vel.publish(cmd_vel);	
			}
		}

		//Set the previous position storgae. This is used to check if the feedback is lost,
		//hence no control update should be performed.
		prevPos = viconPos;


		/**************************************************************
		//CHECK STATUS' 
		//end each loop by checking if the final waypoint has been reached, getting
		//new input data and pausing to limit the control loop rate.
		**************************************************************/
		//Check for new ros messages
		ros::spinOnce();
		rateLimiter.sleep();
		ros::spinOnce();
		
		//If at the last waypoint then land on next loop.
		if (waypoint_info.currentIdx == (waypoint_info.waypointCount-1))
		{
			landingNow = 1;
		}
		
	} 

	if (ros::ok())
	{
		//After main flight loop need to land and shut down
		sleepone.sleep();
		ROS_INFO("flyvslam::LANDING");
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
	}

	ROS_INFO("flyvslam::End flight script.");
  	return 0;
}

//eof
