#ifndef _LQG_CONTROL_H
#define _LQG_CONTROL_H

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

using namespace TooN;

class lqg_control
{
	public:
		//Constructor. No empty constructor available.
		lqg_control(int states,int inputs,int outputs);
		
		//Update the control by one iteration. It is up to the caller to 
		//ensure the rate is appropreate.
		Vector<Dynamic,double> update(Vector<Dynamic,double> inputVec);
		
		//Returns the current internal state
		Vector<Dynamic,double> getState(void);
		
		//Returns the current output
		Vector<Dynamic,double> getOutput(void);
		
		//Functions to set the state-space matrices
		void setA(Matrix<Dynamic,Dynamic,double> inMat);
		void setB(Matrix<Dynamic,Dynamic,double> inMat);
		void setC(Matrix<Dynamic,Dynamic,double> inMat);
		void setD(Matrix<Dynamic,Dynamic,double> inMat);	
		
		//Functions to get the state-space matrices
		Matrix<Dynamic,Dynamic,double> getA(void);
		Matrix<Dynamic,Dynamic,double> getB(void);
		Matrix<Dynamic,Dynamic,double> getC(void);
		Matrix<Dynamic,Dynamic,double> getD(void);	
		
		
		
		//Storage for the system matrices
		Matrix<Dynamic,Dynamic,double>* matA;
		Matrix<Dynamic,Dynamic,double>* matB;
		Matrix<Dynamic,Dynamic,double>* matC;
		Matrix<Dynamic,Dynamic,double>* matD;
			
		//The internal state storage and last input and output
		Vector<Dynamic,double>* currentState;
		Vector<Dynamic,double>* currentInput;
		Vector<Dynamic,double>* currentOutput;
		
		//Store the number of states, inputs and outputs.
		int statesCount;
		int inputsCount;
		int outputsCount;

};


#endif
//eof
