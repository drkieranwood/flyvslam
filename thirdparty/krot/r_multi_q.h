#ifndef _KROT_MULTI_Q_H
#define _KROT_MULTI_Q_H

//Cross-referenced rotation library functions
#include <TooN/TooN.h>  //TooN is required

namespace krot
{	
	//Multiply two quaternions together. 
	//This expands the first quaternion into a special 'quaternion matrix' first, then uses normal matrix multiplication.
	inline TooN::Vector<4,double> r_multi_q(TooN::Vector<4,double> const &quatone,TooN::Vector<4,double> const &quattwo)
	{
		//Turn the first quaternion into a 'quaternion matrix'
		TooN::Matrix<4,4,double> augVecM;
		augVecM(0,0) = quatone[0];
		augVecM(0,1) = (-1.0)*quatone[1];
		augVecM(0,2) = (-1.0)*quatone[2];
		augVecM(0,3) = (-1.0)*quatone[3];

		augVecM(1,0) = quatone[1];
		augVecM(1,1) = quatone[0];
		augVecM(1,2) = (-1.0)*quatone[3];
		augVecM(1,3) = quatone[2];

		augVecM(2,0) = quatone[2];
		augVecM(2,1) = quatone[3];
		augVecM(2,2) = quatone[0];
		augVecM(2,3) = (-1.0)*quatone[1];

		augVecM(3,0) = quatone[3];
		augVecM(3,1) = (-1.0)*quatone[2];
		augVecM(3,2) = quatone[1];
		augVecM(3,3) = quatone[0];

		//Multiply using normal matrix multiplication.
		TooN::Vector<4,double> temp_out = (augVecM*quattwo);
		return temp_out;
	}
	
}

#endif
//eof
