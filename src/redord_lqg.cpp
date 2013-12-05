#include <flyvslam/redord_lqg.h>

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
redord_lqg::redord_lqg(int states,int inputs,int outputs,int ndel)
{
	//Store the state-space sizes.
	this->staCount = states;
	this->inpCount = inputs;
	this->outCount = outputs;
	this->delCount = ndel;
	
	//Initialise the matrices to be the appropriate sizes as per the 
	//given arguments. The {} brackets are used to ensure the automatic
	//destruction of temporary variables.
	{
		Matrix<Dynamic,Dynamic,double> tempMat(states,states);
		matAd = new Matrix<Dynamic,Dynamic,double>(tempMat);
	}
	{
		Matrix<Dynamic,Dynamic,double> tempMat(states,inputs);
		matBd1 = new Matrix<Dynamic,Dynamic,double>(tempMat);
	}
	{
		Matrix<Dynamic,Dynamic,double> tempMat(states,inputs);
		matBd2 = new Matrix<Dynamic,Dynamic,double>(tempMat);
	}
	{
		Matrix<Dynamic,Dynamic,double> tempMat(outputs,states);
		matCd = new Matrix<Dynamic,Dynamic,double>(tempMat);
	}
	
	{
		Matrix<Dynamic,Dynamic,double> tempMat(inputs,states+ndel+1);
		matKc = new Matrix<Dynamic,Dynamic,double>(tempMat);
	}
	
	{
		Matrix<Dynamic,Dynamic,double> tempMat(states,outputs);
		matLc = new Matrix<Dynamic,Dynamic,double>(tempMat);
	}
	
	
	//Initialise the state, input, and output storage vectors to be the 
	//appropriate sizes as per the given arguments.
	{
		Vector<Dynamic,double> tempVec(states);
		currentStateEstimate = new Vector<Dynamic,double>(tempVec);
		*currentStateEstimate = Zeros;
	}
	{
		Vector<Dynamic,double> tempVec(states);
		currentStatePrediction = new Vector<Dynamic,double>(tempVec);
		*currentStatePrediction = Zeros;
	}
	{
		Vector<Dynamic,double> tempVec(inputs);
		currentErrorVec = new Vector<Dynamic,double>(tempVec);
		*currentErrorVec = Zeros;
	}
	{
		Vector<Dynamic,double> tempVec(outputs);
		currentSystemInput = new Vector<Dynamic,double>(tempVec);
		*currentSystemInput = Zeros;
	}
	{
		Vector<Dynamic,double> tempVec(ndel+2);
		pastInputStorage = new Vector<Dynamic,double>(tempVec);
		*pastInputStorage = Zeros;
	}
	{
		Vector<Dynamic,double> tempVec(states+ndel+1);
		currentAugmentedState = new Vector<Dynamic,double>(tempVec);
		*currentAugmentedState = Zeros;
	}
	
} //~Constructor


// Update
Vector<Dynamic,double> redord_lqg::update(Vector<Dynamic,double> errVec, Vector<Dynamic,double> refVec)
{
	//The reduced order update proccess is slightly more complex than a standard
	//state-space update. It requires the 
	

	//1) CORRECT previous prediction with latest measurement (errVec)
	*currentStateEstimate = (*currentStatePrediction) + ((*matLc)*(errVec)) - ( (*matLc)*((*matCd)*(*currentStatePrediction)) );
	

	//2) CONTROL calculation
	//Create augmented matrix
	{
		//Create a temp vector to copy the augmented values into. Also create a 
		//tempvector for the current steatestimate and delay storage.
		Vector<Dynamic,double> augSta(staCount+delCount+1);
		Vector<Dynamic,double> tempEstState(staCount);
		Vector<Dynamic,double> tempDelStore(delCount+2);
		tempEstState = *currentStateEstimate;
		tempDelStore = *pastInputStorage;
		
		//Copy all of the estimated states
		for (int ii=0;ii<staCount;ii++)
		{
			augSta[ii] = tempEstState[ii];
		}
		
		//Copy all of the top n+1 delayed input states
		int jj=1;
		for (int ii=staCount;ii<(staCount+delCount+1);ii++)
		{
			augSta[ii] = tempDelStore[jj];
			jj++;
		}
		
		//Use augmented state in control calculation. Find the error and multiply by the gain Kc
		*currentSystemInput = (*matKc)*(refVec - augSta);
	}

	//3)BOUND the input. This makes sure the model and drone receive the same 
	//actual inputs.
	{
		Vector<Dynamic,double> tempvec(outCount);
		tempvec = *currentSystemInput;
		if (tempvec[0] > maxCtrl)
		{
			tempvec[0] = maxCtrl;
		}
		if (tempvec[0] < minCtrl)
		{
			tempvec[0] = minCtrl;
		}
		
		//Overwrite the current output with the bounded value.
		//This is the value returned by the controller and sent to the drone.
		*currentSystemInput = tempvec;
	}
	
	//4)UPDATE the delay storage. Move all delays along one value and add
	//new control to bottom.
	{
		//Make a copy of the current storage, and an empty tempvec for the new storage
		Vector<Dynamic,double> tempvecold(delCount+2);
		tempvecold = *pastInputStorage;
		Vector<Dynamic,double> tempvecnew(delCount+2);
		for (int ii=0;ii<(delCount+1);ii++)
		{
			tempvecnew[ii] = tempvecold[ii+1];			
		}
		//Add the final element by extracting the value of the current system input.
		Vector<Dynamic,double> tempvec(outCount);
		tempvec = *currentSystemInput;
		tempvecnew[delCount+1] = tempvec[0];
		
		//Over write the delay storage so it is now updated.
		*pastInputStorage = tempvecnew;					
	}
	
	//5)PREDICT the next extimate
	{
		//Move the oldest and 2nd oldest into their own single element vectors (tempoldest,tempnewest)
		//Make a copy of the past storage to be able to access all of its elements.
		Vector<Dynamic,double> tempvec(delCount+2);
		tempvec = *pastInputStorage;
		
		//Make once cell vectors as the inputs (this make the matrix multiplications below work).
		//Use the top two elements of the past input storage (cells 0 and 1)
		Vector<Dynamic,double> tempoldest(1);
		tempoldest[0] = tempvec[0];
		Vector<Dynamic,double> tempnewest(1);
		tempnewest[0] = tempvec[1];
		*currentStatePrediction = ((*matAd)*(*currentStateEstimate)) + ((*matBd1)*(tempoldest)) + ((*matBd2)*(tempnewest));
	}
	
	return *currentSystemInput;
		
} //~update


//Returns the current state.
Vector<Dynamic,double> redord_lqg::getStateEstimate(void)
{
	//Return the output. De-reference the pointer so it is returned as 
	//a dynamic Vector object.
	return *currentStateEstimate;
}


//Returns the current output. i.e the latest control signal
Vector<Dynamic,double> redord_lqg::getSystemInput(void)
{
	//Return the output. De-reference the pointer so it is returned as 
	//a dynamic Vector object.
	return *currentSystemInput;	
}


//Functions to setup the state space matrices. The arguments must be 
//TooN dynamic matrix objects passed by value.
void redord_lqg::setAd(Matrix<Dynamic,Dynamic,double> inMat)
{
	*matAd = inMat;	
}
void redord_lqg::setBd1(Matrix<Dynamic,Dynamic,double> inMat)
{
	*matBd1 = inMat;	
}
void redord_lqg::setBd2(Matrix<Dynamic,Dynamic,double> inMat)
{
	*matBd2 = inMat;	
}
void redord_lqg::setCd(Matrix<Dynamic,Dynamic,double> inMat)
{
	*matCd = inMat;	
}
void redord_lqg::setKc(Matrix<Dynamic,Dynamic,double> inMat)
{
	*matKc = inMat;	
}
void redord_lqg::setLc(Matrix<Dynamic,Dynamic,double> inMat)
{
	*matLc = inMat;	
}

void redord_lqg::setMaxCtrl(double inMax)
{
	maxCtrl = inMax;
}

void redord_lqg::setMinCtrl(double inMin)
{
	minCtrl = inMin;
}

//Functions to retrieve the state space matrices as TooN dynamic matrix 
//objects. The pointers are de-referenced to return by value.
Matrix<Dynamic,Dynamic,double> redord_lqg::getAd(void)
{
	return *matAd;
}
Matrix<Dynamic,Dynamic,double> redord_lqg::getBd1(void)
{
	return *matBd1;
}
Matrix<Dynamic,Dynamic,double> redord_lqg::getBd2(void)
{
	return *matBd2;
}
Matrix<Dynamic,Dynamic,double> redord_lqg::getCd(void)
{
	return *matCd;
}
Matrix<Dynamic,Dynamic,double> redord_lqg::getKc(void)
{
	return *matKc;
}
Matrix<Dynamic,Dynamic,double> redord_lqg::getLc(void)
{
	return *matLc;
}

//Functions to get sizes and rate
int redord_lqg::getNumStates(void)
{
	return staCount;
}
int redord_lqg::getNumInputs(void)
{
	return inpCount;
}
int redord_lqg::getNumOutputs(void)
{
	return outCount;
}
int redord_lqg::getNumDel(void)
{
	return delCount;
}
double redord_lqg::getRate(void)
{
	return ctrlRate;
}


//Function to create a new object and load the matrix values from file.
//All values are also written to the rosout stream. This is a little
//cluttered but ensures each flight logs the controller used.
redord_lqg * loadRedordLQG(std::string filename)
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
		int numDel = 0;
		double rate = 0.0;
		myfile >> numSta >> numInp >> numOut >> numDel >> rate;
		ROS_INFO("flyvslam::[States,Inputs,Outputs,Delays,Rate] = [%d,%d,%d,%d,%10.5f]",numSta,numInp,numOut,numDel,rate);
		
		//Use the sizes to initialise a new lqg_control object
		redord_lqg * newCtrl = new redord_lqg(numSta,numInp,numOut,numDel);
		newCtrl->ctrlRate=rate;
		
		//Iterate through the rest of the file to fill in the matrix
		//values. First read them to a temp variable then copy into 
		//redord_lqg object.
		
		//Ad Ad Ad Ad
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
			newCtrl->setAd(tempMat);
		}
		
		//Bd1 Bd1 Bd1 Bd1
		{
			Matrix<Dynamic,Dynamic,double> tempMat(numSta,numInp);
			for (int ii=0;ii<numSta;ii++)
			{
				for (int jj=0;jj<numInp;jj++)
				{
					double temp = 0.0;
					myfile >> temp;
					tempMat(ii,jj) = temp;
					ROS_INFO("flyvslam::Bd1(%d,%d) = (%10.5f)",ii,jj,temp);
				}
			}
			newCtrl->setBd1(tempMat);
		}
		
		//Bd2 Bd2 Bd2 Bd2
		{
			Matrix<Dynamic,Dynamic,double> tempMat(numSta,numInp);
			for (int ii=0;ii<numSta;ii++)
			{
				for (int jj=0;jj<numInp;jj++)
				{
					double temp = 0.0;
					myfile >> temp;
					tempMat(ii,jj) = temp;
					ROS_INFO("flyvslam::Bd2(%d,%d) = (%10.5f)",ii,jj,temp);
				}
			}
			newCtrl->setBd2(tempMat);
		}
		
		//Cd Cd Cd Cd
		{
			Matrix<Dynamic,Dynamic,double> tempMat(numOut,numSta);
			for (int ii=0;ii<numOut;ii++)
			{
				for (int jj=0;jj<numSta;jj++)
				{
					double temp = 0.0;
					myfile >> temp;
					tempMat(ii,jj) = temp;
					ROS_INFO("flyvslam::Cd(%d,%d) = (%10.5f)",ii,jj,temp);
				}
			}
			newCtrl->setCd(tempMat);
		}
		
		//Kc Kc Kc Kc
		{
			Matrix<Dynamic,Dynamic,double> tempMat(numInp,(numSta+1+numDel));
			for (int ii=0;ii<numInp;ii++)
			{
				for (int jj=0;jj<(numSta+1+numDel);jj++)
				{
					double temp = 0.0;
					myfile >> temp;
					tempMat(ii,jj) = temp;
					ROS_INFO("flyvslam::Kc(%d,%d) = (%10.5f)",ii,jj,temp);
				}
			}
			newCtrl->setKc(tempMat);
		}
		
		//Lc Lc Lc Lc
		{
			Matrix<Dynamic,Dynamic,double> tempMat(numSta,numOut);
			for (int ii=0;ii<numSta;ii++)
			{
				for (int jj=0;jj<numOut;jj++)
				{
					double temp = 0.0;
					myfile >> temp;
					tempMat(ii,jj) = temp;
					ROS_INFO("flyvslam::Lc(%d,%d) = (%10.5f)",ii,jj,temp);
				}
			}
			newCtrl->setLc(tempMat);
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
