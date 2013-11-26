#ifndef _LQG_CONTROL_H
#define _LQG_CONTROL_H

//Required libraries and functions definitions
#include <TooN/TooN.h>
#include <string>


//This is a class that runs a simple state-space system. It accepts 
//updates from an input and modifies the state. It is for a discrete 
//implemetation and the rate of update must be controlled externally.

//1) Upon first creation the object must have its [A B C D] matrices set 
//using setA() setB() ..... 
//2) To update the system with an input use update(), the returned vector 
//is the output
//3) The current output and state are also accessible any time using 
//getOutput(), and getState()

//Note that the definition TooN::Matrix<> or TooN::Vector<> indicates a 
//dynamically sized array.

//Use TooN namespace to reduce the code visual complexity. Any Vector or 
//Matrix objects are TooN versions. The Dynamic sizing is used so the 
//number of states, inputs, and outputs are defined at run time.
using namespace TooN;


class lqg_control
{
	public:
		//Constructor with state-space sizes. No empty constructor available.
		//states  -> Number of states
		//inputs  -> Number of inputs
		//outputs -> Number of outputs
		lqg_control(int states,int inputs,int outputs);
		
		//Update the control by one iteration. It is up to the caller to 
		//ensure the rate is appropriate.
		//inputVec -> The new/current state-space input vector.
		Vector<Dynamic,double> update(Vector<Dynamic,double> inputVec);
		
		//Returns the current internal state.
		Vector<Dynamic,double> getState(void);
		
		//Returns the current output.
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
		
		
		//Functions to get sizes and rate
		int getNumStates(void);
		int getNumInputs(void);
		int getNumOutputs(void);
		int getRate(void);
		
		
		double ctrlRate;
		
		
	private:
		//Storage for the system matrices. They are pointers because the
		//size is unknown until run-time when they are initialised.
		Matrix<Dynamic,Dynamic,double>* matA;
		Matrix<Dynamic,Dynamic,double>* matB;
		Matrix<Dynamic,Dynamic,double>* matC;
		Matrix<Dynamic,Dynamic,double>* matD;
			
		//The internal state storage and last input and output.
		Vector<Dynamic,double>* currentState;
		Vector<Dynamic,double>* currentInput;
		Vector<Dynamic,double>* currentOutput;
		
		//Store the number of states, inputs and outputs.
		int staCount;
		int inpCount;
		int outCount;
};



//Function to load a set of control gains from the specified file into 
//a lqg_control object. This function has an accompanying Matlab script
//which creates the gain files. When one is updated the other sould be 
//also. This function returns a pointer to a new object created within 
//the function.
lqg_control * loadControlGainsFromFile(std::string filename);


#endif
//eof
