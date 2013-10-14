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
#include <r_wrap_pi.h>
#include <r_e_to_q.h>
#include <r_apply_q.h>
#define PI 3.14159265358979323846


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
	int ptamControlOn = 1;
	int avgRollPitchCorr_on = 1;
	int LQG_OK=0;
	
	//Create objects to store and handle vicon, ptam, and waypoint data.
	vicon_data vicon_info;
	ptam_data ptam_info;
	waypoint_data waypoint_info;
	
	//Create control objects for X,Y,Z,W(yaw)
	//Note the LQG is applied using the H2 method since it creates a 
	//single state-space controller. The arguments are the number of [states,inputs,outputs]
	
	int sX=2;
	int iX=1;
	int oX=1;
	
	int sY=2;
	int iY=1;
	int oY=1;
	
	int sZ=2;
	int iZ=1;
	int oZ=1;
	
	int sW=2;
	int iW=1;
	int oW=1;
	
	lqg_control controlX(sX,iX,oX);
	lqg_control controlY(sY,iY,oY);
	lqg_control controlZ(sZ,iZ,oZ);
	lqg_control controlW(sW,iW,oW);
	
	//Set the matrices for the controllers. 
	//Temporary matrices need to be created in order to set the rows and columns.
	{
		TooN::Matrix<TooN::Dynamic,TooN::Dynamic,double> tempMatA(sX,sX);
		tempMatA(0,0) = 0.0;
		tempMatA(0,1) = 0.0;
		tempMatA(1,0) = 1.0;
		tempMatA(1,1) = 0.0;
		controlX.setA(tempMatA);
		
		TooN::Matrix<TooN::Dynamic,TooN::Dynamic,double> tempMatB(sX,iX);
		tempMatB(0,0) = 1.0;
		tempMatB(1,0) = 0.0;
		controlX.setB(tempMatB);
		
		TooN::Matrix<TooN::Dynamic,TooN::Dynamic,double> tempMatC(oX,sX);
		tempMatC(0,0) = 0.0;
		tempMatC(0,1) = 1.0;
		controlX.setC(tempMatC);
		
		TooN::Matrix<TooN::Dynamic,TooN::Dynamic,double> tempMatD(oX,iX);
		tempMatD(0,0) = 0.0;
		controlX.setD(tempMatD);
	}
	
	{
		TooN::Matrix<TooN::Dynamic,TooN::Dynamic,double> tempMatA(sY,sY);
		tempMatA(0,0) = 0.0;
		tempMatA(0,1) = 0.0;
		tempMatA(1,0) = 1.0;
		tempMatA(1,1) = 0.0;
		controlY.setA(tempMatA);
		
		TooN::Matrix<TooN::Dynamic,TooN::Dynamic,double> tempMatB(sY,iY);
		tempMatB(0,0) = 1.0;
		tempMatB(1,0) = 0.0;
		controlY.setB(tempMatB);
		
		TooN::Matrix<TooN::Dynamic,TooN::Dynamic,double> tempMatC(oY,sY);
		tempMatC(0,0) = 0.0;
		tempMatC(0,1) = 1.0;
		controlY.setC(tempMatC);
		
		TooN::Matrix<TooN::Dynamic,TooN::Dynamic,double> tempMatD(oY,iY);
		tempMatD(0,0) = 0.0;
		controlY.setD(tempMatD);
	}
	
	{
		TooN::Matrix<TooN::Dynamic,TooN::Dynamic,double> tempMatA(sZ,sZ);
		tempMatA(0,0) = 0.0;
		tempMatA(0,1) = 0.0;
		tempMatA(1,0) = 1.0;
		tempMatA(1,1) = 0.0;
		controlZ.setA(tempMatA);
		
		TooN::Matrix<TooN::Dynamic,TooN::Dynamic,double> tempMatB(sZ,iZ);
		tempMatB(0,0) = 1.0;
		tempMatB(1,0) = 0.0;
		controlZ.setB(tempMatB);
		
		TooN::Matrix<TooN::Dynamic,TooN::Dynamic,double> tempMatC(oZ,sZ);
		tempMatC(0,0) = 0.0;
		tempMatC(0,1) = 1.0;
		controlZ.setC(tempMatC);
		
		TooN::Matrix<TooN::Dynamic,TooN::Dynamic,double> tempMatD(oZ,iZ);
		tempMatD(0,0) = 0.0;
		controlZ.setD(tempMatD);
	}
	
	
	{
		TooN::Matrix<TooN::Dynamic,TooN::Dynamic,double> tempMatA(sW,sW);
		tempMatA(0,0) = 0.0;
		tempMatA(0,1) = 0.0;
		tempMatA(1,0) = 1.0;
		tempMatA(1,1) = 0.0;
		controlW.setA(tempMatA);
		
		TooN::Matrix<TooN::Dynamic,TooN::Dynamic,double> tempMatB(sW,iW);
		tempMatB(0,0) = 1.0;
		tempMatB(1,0) = 0.0;
		controlW.setB(tempMatB);
		
		TooN::Matrix<TooN::Dynamic,TooN::Dynamic,double> tempMatC(oW,sW);
		tempMatC(0,0) = 0.0;
		tempMatC(0,1) = 1.0;
		controlW.setC(tempMatC);
		
		TooN::Matrix<TooN::Dynamic,TooN::Dynamic,double> tempMatD(oW,iW);
		tempMatD(0,0) = 0.0;
		controlW.setD(tempMatD);
	}
	
	
	
/*
    //This is to test the state-space equations are being evaluated correctly.
    //The update function is called 5 times with a single input [1,2,3,4,5]
	TooN::Vector<TooN::Dynamic,double> tempInp(1);
	TooN::Vector<TooN::Dynamic,double> tempOut(1);
	tempInp = TooN::makeVector(1.0);
	tempOut = controlX.update(tempInp);
	std::cout << tempInp[0] << " " << tempOut[0] << std::endl;
	
	tempInp = TooN::makeVector(2.0);
	tempOut = controlX.update(tempInp);
	std::cout << tempInp[0] << " " << tempOut[0] << std::endl;
	
	tempInp = TooN::makeVector(3.0);
	tempOut = controlX.update(tempInp);
	std::cout << tempInp[0] << " " << tempOut[0] << std::endl;
	
	tempInp = TooN::makeVector(4.0);
	tempOut = controlX.update(tempInp);
	std::cout << tempInp[0] << " " << tempOut[0] << std::endl;

	tempInp = TooN::makeVector(5.0);
	tempOut = controlX.update(tempInp);
	std::cout << tempInp[0] << " " << tempOut[0] << std::endl;
	

	//This is to check the LQG control matrices were being set correctly.
	//The setM() command is used above and the getM() command is used here.
	//Note how the get() command sucessfully sizes the tempM dynamic matrices.
	TooN::Matrix<TooN::Dynamic,TooN::Dynamic,double> tempA = controlX.getA();
	std::cout << tempA(0,0) << " " << tempA(0,1) << " " << tempA(0,2) << std::endl;
	//std::cout << tempA(1,0) << " " << tempA(1,1) << " " << tempA(1,2) << std::endl;
	//std::cout << tempA(2,0) << " " << tempA(2,1) << " " << tempA(2,2) << std::endl;
	
	TooN::Matrix<TooN::Dynamic,TooN::Dynamic,double> tempB = controlX.getB();
	std::cout << tempB(0,0) << " " << tempB(1,0) << " " << tempB(2,0) << std::endl;
	
	TooN::Matrix<TooN::Dynamic,TooN::Dynamic,double> tempC = controlX.getC();
	std::cout << tempC(0,0) << " " << tempC(0,1) << " " << tempC(0,2) << std::endl;
	
	TooN::Matrix<TooN::Dynamic,TooN::Dynamic,double> tempD = controlX.getD();
	std::cout << tempD(0,0) << std::endl;
*/

	/*
	{
		TooN::Matrix<3,3,double> tempMatA =  TooN::Data(0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0);
		controlY.setA(tempMatA);
		TooN::Matrix<3,1,double> tempMatB =  TooN::Data(0.0,0.0,0.0);
		controlY.setB(tempMatB);
		TooN::Matrix<1,3,double> tempMatC =  TooN::Data(0.0,0.0,0.0);
		controlY.setC(tempMatC);
		TooN::Matrix<1,1,double> tempMatD =  TooN::Data(0.0);
		controlY.setD(tempMatD);
	}
	{
		TooN::Matrix<3,3,double> tempMatA =  TooN::Data(0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0);
		controlZ.setA(tempMatA);
		TooN::Matrix<3,1,double> tempMatB =  TooN::Data(0.0,0.0,0.0);
		controlZ.setB(tempMatB);
		TooN::Matrix<1,3,double> tempMatC =  TooN::Data(0.0,0.0,0.0);
		controlZ.setC(tempMatC);
		TooN::Matrix<1,1,double> tempMatD =  TooN::Data(0.0);
		controlZ.setD(tempMatD);
	}
	{
		TooN::Matrix<3,3,double> tempMatA =  TooN::Data(0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0);
		controlX.setA(tempMatA);
		TooN::Matrix<3,1,double> tempMatB =  TooN::Data(0.0,0.0,0.0);
		controlX.setB(tempMatB);
		TooN::Matrix<1,3,double> tempMatC =  TooN::Data(0.0,0.0,0.0);
		controlX.setC(tempMatC);
		TooN::Matrix<1,1,double> tempMatD =  TooN::Data(0.0);
		controlX.setD(tempMatD);
	}
	*/
	

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

	//Flags to control loops, flow, and initialisations
	int PTAM_OK = 0;			//Flag to break out of PTAM control back to the original, very stable, Vicon safety controller. 1=PTAM control, 0=Vicon control.
	int landingNow = 0;			//Flag to indicate the last waypoint has been reached and landing should occur. Can be used by other code to induce a landing at any time.
	int ptamInit = 0;			//Flag to control the PTAM baseline initilisation. When complete this is set to 2.
	int scaleInit = 0;			//Flag to control the PTAM scale initilisation. When complete this is set to 2.
	int stepOn = 0;
	double ptamViconTol = 0.2;  //Toleracne between Vicon and PTAM positions
	int ptamCheck = 0;
	int takeOffInit = 0;
	
	//Storage for sample values used to calculate the scale. 
	TooN::Vector<3, double> ptamPos_one;
	TooN::Vector<3, double> ptamPos_two;
	TooN::Vector<3, double> viconPos_one;
	TooN::Vector<3, double> viconPos_two;
	
	//The average roll and pitch and count of included measurements
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
	ros::Publisher  pub_cmd_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

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


	//=========================
	//Timings
	//=========================

	//The timing is controled by a ros::rate object. The argument is the desired loop rate in Hz. 
	//Note this must be faster than the waypoint transition times.
	ros::Rate rateLimiter(100);
	
	//Get the waypoints from file, and set the flight start time
	waypoint_info.readWaypointData();
	waypoint_info.start_time = ros::Time::now();
	waypoint_info.waypoint_time = ros::Time::now();

	//Send the takeoff command after a 2 second delay. To allow all ROS init. processes to complete.
	ros::Duration sleepone = ros::Duration(1,0);
	sleepone.sleep();
	sleepone.sleep();
	
	//If not using the average roll and pitch method then take-off now.
	if (avgRollPitchCorr_on==0)
	{
		ROS_INFO("flyvslam::TAKEOFF");
		{
			std_msgs::Empty tempMsg;
			pub_takeoff.publish(tempMsg);
		}		
	}
	
	//Main loop. Loop until last waypoint, then land.
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
		//These will be zeros for the first part of the flight, then arbirarily scaled, then become NED once the scale has been set.
		ptamPos = ptam_info.currentPos;
		ptamRot = ptam_info.currentRot;
		ptamVel = ptam_info.currentVel;
		ptamYaw = ptam_info.currentYaw;


		{
			//Output the reference, Vicon, and PTAM, data as messages
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
		//PTAM STUFF
		//at the start of the flight two motions need perforing,
		//1) move to set a baseline for PTAM
		//2) move to find a scale for PTAM
		**************************************************************/
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
			
			if (avgRollPitchCorr_on==1)
			{
				//Set a manual pose correction. The orientation correction is set to do nothing at the moment. 
				//Hence the orientation output will be in the camera frame.
				//This overwrites the one set by the setinitVicon() above.
				TooN::Vector<3,double> initPosTemp = TooN::makeVector(0.5,-0.195,-1.0);
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
			ptam_info.setPtamScale(double(viconDist/ptamDist));
			
			//If using the no Vicon averaging method overwrtie the scale 
			//using the dead reckoning estimate.
			if (avgRollPitchCorr_on==1)
			{
				ptam_info.setPtamScale(double(2.0/ptamDist));
			}
			scaleInit = 2;
		}
	    //If at the ninth waypoint (idx==8) then takeoff.
		if((waypoint_info.currentIdx==8) && (takeOffInit==0))
		{
			//If using the averaging method then this is the time to takeoff.
			if (avgRollPitchCorr_on==1)
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
		if (ptamInit==2 && avgCount<50)
		{
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
		
		
		/**************************************************************
		//CONTROL SELECTION
		//check if the PTAM data is near the Vicon data, if so then use the PTAM
		//based controller, else break into the emergency controller.
		**************************************************************/
		
		//Check if both the initilisations have been completed for PTAM. 
		//Only think about activating PTAM control, once they are complete.
		if (ptamInit==2 && scaleInit==2 && (ptamControlOn))
		{
			if (PTAM_OK!=1)
			{
					ROS_INFO("flyvslam::PTAM Control active");
					PTAM_OK = 1;
			}	
		}

		if (PTAM_OK==1) //use PTAM controller. If inside the PTAM controller the PTAM_OK=0 flag is set, then the Vicon control will over-write.
		{
			/**************************************************************
			//PTAM CONTROL
			//use the incomming PTAM data in an LQG or Hinf controller. This loop rate is 100Hz 
			//but rate throttling for PTAM is achieved by not updating the PTAM pose quickly, and 
			//checking if the data has changed between loops.
			**************************************************************/
			//HACK:: if ptam data is ok then replace the vicon data with it.
			//This means the controler below will use the ptam data
			viconPos = ptamPos;
			viconVel = ptamVel;
			viconYaw = ptamYaw;
			viconRot = ptamRot;
			
		} //PTAM_OK==1


		if (1)
		{
			/**************************************************************
			//VICON CONTROL
			//this is a safety controller used to rescue the MAV when things go wrong.
			//to use it just set PTAM_OK=0 anywhere above and this control will over-write
			//any previous one.
			**************************************************************/

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
			if (PTAM_OK==1)
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
			if (PTAM_OK==1)
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
			//Transform and output
			//========================		

			//Axis transform from Vicon to drone (z remains the same but is not used)
			TooN::Vector<3, double> drone_axis_x  = TooN::makeVector(cos(viconYaw), sin(viconYaw),0); 
			TooN::Vector<3, double> drone_axis_y  = TooN::makeVector(-sin(viconYaw), cos(viconYaw),0);
			TooN::Vector<3, double> tmp_vel_drone = TooN::makeVector(tmp_vel*drone_axis_x,tmp_vel*drone_axis_y,0);
				
			if (ptamControlOn==0)
			{					
				//Publish movement commands to drones ros topic.
				//This is only performed if new Vicon data has been received in the last 1 second.
				if( (1.0) > ((ros::Time::now()-(vicon_info.vicon_last_update_time)).toSec()) )
				{
					geometry_msgs::Twist cmd_vel;
					cmd_vel.linear.x = tmp_vel_drone[0];
					cmd_vel.linear.y = (-1)*tmp_vel_drone[1];		//Axis negated to make NED
					cmd_vel.linear.z = (-1)*tmp_Z_cmd;				//Axis negated to make NED
					cmd_vel.angular.z = (-1)*angErr;				//Axis negated to make NED
					pub_cmd_vel.publish(cmd_vel);	
				}
				else
				{
					//Else no new Vicon data is available so send zeros
					ROS_INFO("flyvslam::No new Vicon data or nan error. Sending [0,0,0,0]' vel_cmd");
					geometry_msgs::Twist cmd_vel;
					cmd_vel.linear.x = 0;
					cmd_vel.linear.y = 0;
					cmd_vel.linear.z = 0;
					cmd_vel.angular.z = 0;
					pub_cmd_vel.publish(cmd_vel);	
				}
			}
			else
			{
				//If using ptam control then always publish the command
				geometry_msgs::Twist cmd_vel;
				cmd_vel.linear.x = tmp_vel_drone[0];
				cmd_vel.linear.y = (-1)*tmp_vel_drone[1];		//Axis negated to make NED
				cmd_vel.linear.z = (-1)*tmp_Z_cmd;				//Axis negated to make NED
				cmd_vel.angular.z = (-1)*angErr;				//Axis negated to make NED
				pub_cmd_vel.publish(cmd_vel);		
			}
			
		}
		else if (LQG_OK==1)
		{
			/**************************************************************
			//LQG/H2 Control
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
				tempOutputX = controlX.update(tempInput);
			}
			{
				TooN::Vector<TooN::Dynamic,double> tempInput(iY);
				tempInput = TooN::makeVector(mavPosErr[1]);
				tempOutputX = controlY.update(tempInput);
			}
			{
				TooN::Vector<TooN::Dynamic,double> tempInput(iZ);
				tempInput = TooN::makeVector(mavPosErr[2]);
				tempOutputX = controlZ.update(tempInput);
			}
			{
				TooN::Vector<TooN::Dynamic,double> tempInput(iW);
				tempInput = TooN::makeVector(nedYawErr);
				tempOutputX = controlW.update(tempInput);
			}
			
			
			//Fill in the control values to be sent to the MAV and send.
			geometry_msgs::Twist cmd_vel;
			cmd_vel.linear.x  = tempOutputX[0];
			cmd_vel.linear.y  = (-1)*tempOutputY[0];		//Axis negated to make NED
			cmd_vel.linear.z  = (-1)*tempOutputZ[0];		//Axis negated to make NED
			cmd_vel.angular.z = (-1)*tempOutputW[0];		//Axis negated to make NED
			pub_cmd_vel.publish(cmd_vel);	
			
		}


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
