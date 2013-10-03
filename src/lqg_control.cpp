#include <flyvslam/lqg_control.h>

using namespace TooN;

lqg_control::lqg_control(int states,int inputs,int outputs)
{
	//The default system is a SISO pass-through. i.e the system has no
	//dynamics and outputs the input immidiately.
	//The class defines a set of pointers to dynamic matrices.
	//These are now initialised with an empty 1x1 matirx.
	statesCount  = states;
	inputsCount  = inputs;
	outputsCount = outputs;
	
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
	
	//States inputs and outputs
	{
		Vector<Dynamic,double> tempVec(statesCount);
		currentState = new Vector<Dynamic,double>(tempVec);
		*currentState = Zeros;
	}
	{
		Vector<Dynamic,double> tempVec(inputsCount);
		currentInput = new Vector<Dynamic,double>(tempVec);
		*currentInput = Zeros;
	}
	{
		Vector<Dynamic,double> tempVec(outputsCount);
		currentOutput = new Vector<Dynamic,double>(tempVec);
		*currentOutput = Zeros;
	}
}

Vector<Dynamic,double> lqg_control::update(Vector<Dynamic,double> inputVec)
{
	//Create the output
	//y[k] = C*x[k] + D*u[k]
	*currentOutput = ((*matC)*(*currentState)) + ((*matD)*(inputVec));
	
	//Update the state
	//x[k+1] = A*x[k] + B*u[k]
	*currentState = ((*matA)*(*currentState)) + ((*matB)*(inputVec));;
	
	//Update the input storage
	*currentInput = inputVec;
	
	//Return the output. De-reference the pointer so it is returned as 
	//a dynamic Vector object.
	return *currentOutput;
}

//Returns the current state
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

//Functions to setup the state space matrices
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


//Functions to retrieve the state space matrices
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

//eof
