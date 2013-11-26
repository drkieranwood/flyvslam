#include <flyvslam/lqg_control.h>

//Required libraries and functions definitions
#include <string>
#include <iostream>
#include <fstream>
#include "ros/ros.h"


//Use TooN namespace to reduce the code visual complexity. Any Vector or 
//Matrix objects are TooN versions. The Dynamic sizing is used so the 
//number of states, inputs, and outputs are defined at run time.
using namespace TooN;
#include "ros/ros.h"

//Constructor
lqg_control::lqg_control(int states,int inputs,int outputs)
{
	//Store the state-space sizes.
	this->staCount  = states;
	this->inpCount  = inputs;
	this->outCount  = outputs;
	
	//Initialise the matrices to be the appropriate sizes as per the 
	//given arguments. The {} brackets are used to ensure the automatic
	//destruction of temporary variables.
	{
		Matrix<Dynamic,Dynamic,double> tempMat(states,states);
		matA = new Matrix<Dynamic,Dynamic,double>(tempMat);
	}
	{
		Matrix<Dynamic,Dynamic,double> tempMat(states,inputs);
		matB = new Matrix<Dynamic,Dynamic,double>(tempMat);
	}
	{
		Matrix<Dynamic,Dynamic,double> tempMat(outputs,states);
		matC = new Matrix<Dynamic,Dynamic,double>(tempMat);
	}
	{
		Matrix<Dynamic,Dynamic,double> tempMat(outputs,inputs);
		matD = new Matrix<Dynamic,Dynamic,double>(tempMat);
	}
	
	
	//Initialise the state, input, and output storage vectors to be the 
	//appropriate sizes as per the given arguments.
	{
		Vector<Dynamic,double> tempVec(states);
		currentState = new Vector<Dynamic,double>(tempVec);
		*currentState = Zeros;
	}
	{
		Vector<Dynamic,double> tempVec(inputs);
		currentInput = new Vector<Dynamic,double>(tempVec);
		*currentInput = Zeros;
	}
	{
		Vector<Dynamic,double> tempVec(outputs);
		currentOutput = new Vector<Dynamic,double>(tempVec);
		*currentOutput = Zeros;
	}
	
} //~Constructor


// Update
Vector<Dynamic,double> lqg_control::update(Vector<Dynamic,double> inputVec)
{
	//Create the output
	//y[k] = C*x[k] + D*u[k]
	*currentOutput = ((*matC)*(*currentState)) + ((*matD)*(inputVec));
	
	//Update the state
	//x[k+1] = A*x[k] + B*u[k]
	*currentState = ((*matA)*(*currentState)) + ((*matB)*(inputVec));
	
	//Update the input storage
	*currentInput = inputVec;
	
	//Return the output. De-reference the pointer so it is returned as 
	//a dynamic Vector object.
	return *currentOutput;
	
} //~update


//Returns the current state.
Vector<Dynamic,double> lqg_control::getState(void)
{
	//Return the output. De-reference the pointer so it is returned as 
	//a dynamic Vector object.
	return *currentState;
}


//Returns the current output
Vector<Dynamic,double> lqg_control::getOutput(void)
{
	//Return the output. De-reference the pointer so it is returned as 
	//a dynamic Vector object.
	return *currentOutput;	
}


//Functions to setup the state space matrices. The arguments must be 
//TooN dynamic matrix objects passed by value.
void lqg_control::setA(Matrix<Dynamic,Dynamic,double> inMat)
{
	*matA = inMat;	
}
void lqg_control::setB(Matrix<Dynamic,Dynamic,double> inMat)
{
	*matB = inMat;	
}
void lqg_control::setC(Matrix<Dynamic,Dynamic,double> inMat)
{
	*matC = inMat;	
}
void lqg_control::setD(Matrix<Dynamic,Dynamic,double> inMat)
{
	*matD = inMat;	
}


//Functions to retrieve the state space matrices as TooN dynamic matrix 
//objects. The pointers are de-referenced to return by value.
Matrix<Dynamic,Dynamic,double> lqg_control::getA(void)
{
	return *matA;
}
Matrix<Dynamic,Dynamic,double> lqg_control::getB(void)
{
	return *matB;
}
Matrix<Dynamic,Dynamic,double> lqg_control::getC(void)
{
	return *matC;
}
Matrix<Dynamic,Dynamic,double> lqg_control::getD(void)
{
	return *matD;
}

//Functions to get sizes and rate
int lqg_control::getNumStates(void)
{
	return staCount;
}
int lqg_control::getNumInputs(void)
{
	return inpCount;
}
int lqg_control::getNumOutputs(void)
{
	return outCount;
}
int lqg_control::getRate(void)
{
	return ctrlRate;
}


//Function to create a new object and load the matrix values from file.
//All values are also written to the rosout stream. This is a little
//cluttered but ensures each flight logs the controller used.
lqg_control * loadControlGainsFromFile(std::string filename)
{
	//Temp string to hold each line of text from the file.
	std::string line;
	
	//Attempt to open the file for reading.
	std::ifstream myfile(filename.c_str());

	if (myfile.is_open())
	{
		//If the file opened for reading.
		ROS_INFO("flyvslam::Gain file opened...");
		
		//Get the first four lines relating to the matrix sizes and
		//implementation rate. 
		int numSta = 0;
		int numInp = 0;
		int numOut = 0;
		double rate = 0.0;
		myfile >> numSta >> numInp >> numOut >> rate;
		ROS_INFO("flyvslam::[Sta,Inp,Out,Rate] = [%d,%d,%d,%10.5f]",numSta,numInp,numOut,rate);
		
		//Use the sizes to initialise a new lqg_control object
		lqg_control * newCtrl = new lqg_control(numSta,numInp,numOut);
		newCtrl->ctrlRate=rate;
		
		//Iterate through the rest of the file to fill in the matrix
		//values. Fist read them to a temp variable then copy into 
		//lqg_control object.
		
		//A A A A
		{
			Matrix<Dynamic,Dynamic,double> tempMat(numSta,numSta);
			for (int ii=0;ii<numSta;ii++)
			{
				for (int jj=0;jj<numSta;jj++)
				{
					double temp = 0.0;
					myfile >> temp;
					tempMat(ii,jj) = temp;
					ROS_INFO("flyvslam::A(%d,%d) = (%10.5f)",ii,jj,temp);
				}
			}
			newCtrl->setA(tempMat);
		}
		
		//B B B B
		{
			Matrix<Dynamic,Dynamic,double> tempMat(numSta,numInp);
			for (int ii=0;ii<numSta;ii++)
			{
				for (int jj=0;jj<numInp;jj++)
				{
					double temp = 0.0;
					myfile >> temp;
					tempMat(ii,jj) = temp;
					ROS_INFO("flyvslam::B(%d,%d) = (%10.5f)",ii,jj,temp);
				}
			}
			newCtrl->setB(tempMat);
		}
		
		//C C C C
		{
			Matrix<Dynamic,Dynamic,double> tempMat(numOut,numSta);
			for (int ii=0;ii<numOut;ii++)
			{
				for (int jj=0;jj<numSta;jj++)
				{
					double temp = 0.0;
					myfile >> temp;
					tempMat(ii,jj) = temp;
					ROS_INFO("flyvslam::C(%d,%d) = (%10.5f)",ii,jj,temp);
				}
			}
			newCtrl->setC(tempMat);
		}
		
		//D D D D
		{
			Matrix<Dynamic,Dynamic,double> tempMat(numOut,numInp);
			for (int ii=0;ii<numOut;ii++)
			{
				for (int jj=0;jj<numInp;jj++)
				{
					double temp = 0.0;
					myfile >> temp;
					tempMat(ii,jj) = temp;
					ROS_INFO("flyvslam::D(%d,%d) = (%10.5f)",ii,jj,temp);
				}
			}
			newCtrl->setD(tempMat);
		}
		
		
		//Close the file.
		myfile.close();
		ROS_INFO("flyvslam::Gain file closed.");
		
		//Return the pointer to the newly created and populated object.
		return newCtrl;
	}
	else
	{
		//If the file is not opened, then do not create anything and 
		//return a null pointer.
		ROS_INFO("flyvslam:: Could not open control gain file.");
		return NULL;
	}
}

//eof
