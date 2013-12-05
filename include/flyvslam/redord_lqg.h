#ifndef _REDORD_LQG_H
#define _REDORD_LQG_H

//Required libraries and functions definitions
#include <TooN/TooN.h>
#include <string>


//This is a reduced order state-space LQG style controller. It is used specifically
//for the BF axis control of the ARDrone2.0 with assumed loop delays. The relevant 
//Ad,Bd1,Bd2,Cd,Lc,Kc matrices are all read from file during initialisation and
//all initial internal states are set to zero.
//The update function accepts the current BF error measurement and creates the
//cuurrent BF input for that axis.
using namespace TooN;


class redord_lqg
{
	public:
		//Constructor with state-space sizes. No empty constructor available.
		//states  -> Number of states of non-delayed system
		//inputs  -> Number of inputs (always one for SISO case)
		//outputs -> Number of outputs (always one for SISO case)
		//ndel    -> Number of additional delay states 
		//(n+1 additional states in augmented system, n+2 states in past input buffer)
		redord_lqg(int states,int inputs,int outputs,int ndel);
		
		//Update the control by one iteration. It is up to the caller to 
		//ensure the rate is appropriate.
		//errVec -> The current error expressed in the BF.
		Vector<Dynamic,double> update(Vector<Dynamic,double> errVec, Vector<Dynamic,double> refVec);
		
		//Returns the current state estimate as of the last update.
		Vector<Dynamic,double> getStateEstimate(void);
		
		//Returns the current output. Which is the current real system BF input.
		Vector<Dynamic,double> getSystemInput(void);
		
		//Functions to set the state-space matrices
		void setAd(Matrix<Dynamic,Dynamic,double> inMat);
		void setBd1(Matrix<Dynamic,Dynamic,double> inMat);
		void setBd2(Matrix<Dynamic,Dynamic,double> inMat);
		void setCd(Matrix<Dynamic,Dynamic,double> inMat);
		void setKc(Matrix<Dynamic,Dynamic,double> inMat);
		void setLc(Matrix<Dynamic,Dynamic,double> inMat);
		void setMaxCtrl(double inMax);
		void setMinCtrl(double inMin);
		
		//Functions to get the state-space matrices
		Matrix<Dynamic,Dynamic,double> getAd(void);
		Matrix<Dynamic,Dynamic,double> getBd1(void);
		Matrix<Dynamic,Dynamic,double> getBd2(void);
		Matrix<Dynamic,Dynamic,double> getCd(void);
		Matrix<Dynamic,Dynamic,double> getKc(void);
		Matrix<Dynamic,Dynamic,double> getLc(void);
		
		
		//Functions to get sizes and rate
		int getNumStates(void);
		int getNumInputs(void);
		int getNumOutputs(void);
		int getNumDel(void);
		double getRate(void);
		
		double ctrlRate;
				
	private:
		//Storage for the system matrices. They are pointers because the
		//size is unknown until run-time when they are initialised.
		Matrix<Dynamic,Dynamic,double>* matAd;
		Matrix<Dynamic,Dynamic,double>* matBd1;
		Matrix<Dynamic,Dynamic,double>* matBd2;
		Matrix<Dynamic,Dynamic,double>* matCd;
		Matrix<Dynamic,Dynamic,double>* matKc;
		Matrix<Dynamic,Dynamic,double>* matLc;
			
		//The internal state storage and last input and output.
		Vector<Dynamic,double>* currentStateEstimate;
		Vector<Dynamic,double>* currentStatePrediction;
		Vector<Dynamic,double>* currentErrorVec;
		Vector<Dynamic,double>* currentSystemInput;
		Vector<Dynamic,double>* pastInputStorage;
		Vector<Dynamic,double>* currentAugmentedState;
		
		//Store the number of states, inputs and outputs.
		int staCount;
		int inpCount;
		int outCount;
		int delCount;
		double maxCtrl;
		double minCtrl;
};



//Function to load a set of control gains from the specified file into 
//a redord_lqg object. This function has an accompanying Matlab script
//which creates the gain files. When one is updated the other sould be 
//also. This function returns a pointer to a new object created within 
//the function.
redord_lqg * loadRedordLQG(std::string filename);


#endif
//eof
