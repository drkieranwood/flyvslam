#include <flyvslam/lqg_control.h>

lqg_control::lqg_control()
{
	//The default system is a SISO pass-through. i.e the system has no
	//dynamics and outputs the input immidiately.
	matA = TooN::data(0.0);
	matB = TooN::data(1.0);
	matC = TooN::data(0.0);
	matD = TooN::data(1.0);
	
	currentState  = TooN::makeVector(0.0);
    currentInput  = TooN::makeVector(0.0);
    currentOutput = TooN::makeVector(0.0);
}

TooN::Vector<TooN::Dynamic,double> lqg_control::update(TooN::Vector<TooN::Dynamic,double> inputVec)
{
	//Create the output
	//y[k] = C*x[k] + D*u[k]
	currentOutput = (matC*currentState) + (matD*inputVec);
	
	//Update the state
	//x[k+1] = A*x[k] + B*u[k]
	TooN::Matrix<TooN::Dynamic,TooN::Dynamic,double> tempMat = (matA*currentState) + (matB*inputVec);
	currentState = tempMat;
	
	//Update the input
	currentInput = inputVec;
	
	//Return the output
	return currentOutput;
}

//Returns the current state
TooN::Vector<TooN::Dynamic,double> lqg_control::getState(void)
{
	return currentState;
}
		
//Returns the current output
TooN::Vector<TooN::Dynamic,double> lqg_control::getOutput(void)
{
	return currentOutput;	
}

//Functions to setup the state space matrices

lqg_control::setA(TooN::Matrix<TooN::Dynamic,TooN::Dynamic,double> inMat)
{
	matA = inMat;	
}
lqg_control::setB(TooN::Matrix<TooN::Dynamic,TooN::Dynamic,double> inMat)
{
	matB = inMat;	
}
lqg_control::setC(TooN::Matrix<TooN::Dynamic,TooN::Dynamic,double> inMat)
{
	matC = inMat;	
}
lqg_control::setD(TooN::Matrix<TooN::Dynamic,TooN::Dynamic,double> inMat)
{
	matD = inMat;	
}

//eof
