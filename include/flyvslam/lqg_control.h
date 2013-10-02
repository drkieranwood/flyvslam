#ifndef _LQG_CONTROL_SYSTEM_H
#define _LQG_CONTROL_SYSTEM_H

//Required libraries and functions definitions
#include <TooN/TooN.h>


//This is a class that runsa  sinple state space system. It accepts 
//updates from an input and modifies the state. It is for a discrete 
//implemetation and the rate of update must be controlled externally.

//1) Upon first creation the onject must have its [A B C D] matrices set 
//using setA() setB() ..... 
//2) To update the system with an input use update() the returned vector 
//is the output
//3) The current output and state are also accessible using 
//getOutput(), and getState()

//Note that the definition TooN::Matrix<> or TooN::Vector<> indicates a 
//dynamically sized array.
class lqg_control
{
	public:
		//Constructor.
		lqg_control();
		
		//Update the control by one iteration. It is up to the caller to 
		//ensure the rate is appropreate.
		TooN::Vector<TooN::Dynamic,double> update(TooN::Vector<TooN::Dynamic,double> inputVec);
		
		//Returns the current internal state
		TooN::Vector<TooN::Dynamic,double> getState(void);
		
		//Returns the current output
		TooN::Vector<TooN::Dynamic,double> getOutput(void);
		
		//Functions to set the state-space matrices
		void setA(TooN::Matrix<TooN::Dynamic,TooN::Dynamic,double> inMat);
		void setB(TooN::Matrix<TooN::Dynamic,TooN::Dynamic,double> inMat);
		void setC(TooN::Matrix<TooN::Dynamic,TooN::Dynamic,double> inMat);
		void setD(TooN::Matrix<TooN::Dynamic,TooN::Dynamic,double> inMat);		
		
		
		
		//Storage for the system matrices
		TooN::Matrix<TooN::Dynamic,TooN::Dynamic,double> matA;
		TooN::Matrix<TooN::Dynamic,TooN::Dynamic,double> matB;
		TooN::Matrix<TooN::Dynamic,TooN::Dynamic,double> matC;
		TooN::Matrix<TooN::Dynamic,TooN::Dynamic,double> matD;
		
		//The internal state storage and last input and output
		TooN::Vector<TooN::Dynamic,double> currentState;
		TooN::Vector<TooN::Dynamic,double> currentInput;
		TooN::Vector<TooN::Dynamic,double> currentOutput;
};


#endif
//eof
