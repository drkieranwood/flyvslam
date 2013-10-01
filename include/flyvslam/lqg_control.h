#ifndef _LQG_CONTROL_SYSTEM_H
#define _LQG_CONTROL_SYSTEM_H

//Required libraries and functions definitions
#include <TooN/TooN.h>
#include <TooN/se3.h>
#include <TooN/so3.h>

class lqg_control
{
	public:
		//Constructor.
		lqg_control();
		
		void setA(TooN::Matrix<Dynamic,Dynamic> inMat);
		void setB(TooN::Matrix<Dynamic,Dynamic> inMat);
		void setC(TooN::Matrix<Dynamic,Dynamic> inMat);
		void setD(TooN::Matrix<Dynamic,Dynamic> inMat);		
		
		
		//Storage for the system matrices
		TooN::Matrix<Dynamic,Dynamic> matA;
		TooN::Matrix<Dynamic,Dynamic> matB;
		TooN::Matrix<Dynamic,Dynamic> matC;
		TooN::Matrix<Dynamic,Dynamic> matD;
};


#endif
//eof
