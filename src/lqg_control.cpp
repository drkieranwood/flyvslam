#include <flyvslam/lqg_control.h>




lqg_control::lqg_control()
{
	//The default system is a SISO pass-through. i.e the system has no
	//dynamics and outputs the input immidiately.
	matA = data(1.0);
	matB = data(1.0);
	matC = data(0.0);
	matD = data(1.0);
}

lqg_control::setA(TooN::Matrix<Dynamic,Dynamic> inMat)
{
	matA = inMat;	
}
lqg_control::setB(TooN::Matrix<Dynamic,Dynamic> inMat)
{
	matB = inMat;	
}
lqg_control::setC(TooN::Matrix<Dynamic,Dynamic> inMat)
{
	matC = inMat;	
}
lqg_control::setD(TooN::Matrix<Dynamic,Dynamic> inMat)
{
	matD = inMat;	
}
