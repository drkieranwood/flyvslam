#ifndef _KROT_APPLY_Q_H
#define _KROT_APPLY_Q_H

//Cross-referenced rotation library functions
#include <TooN/TooN.h>
#include <r_check_q.h>


//NOTE:: the TooN library is used and must be included in the compile search path.

namespace krot
{
	//Apply a quaternion to a vector
	static TooN::Vector<3,double> r_apply_q(TooN::Vector<3,double> tempVec,TooN::Vector<4,double> tempQuat)
	{
		//Check the quaternion
		r_check_q(tempQuat);

		//Construct an augmented vector
		TooN::Vector<4,double> augVec = TooN::makeVector(0.0,tempVec[0],tempVec[1],tempVec[2]);

		//Turn the vector into matrix
		TooN::Matrix<4,4,double> augVecM;
		augVecM(0,0) = augVec[0];
		augVecM(0,1) = (-1.0)*augVec[1];
		augVecM(0,2) = (-1.0)*augVec[2];
		augVecM(0,3) = (-1.0)*augVec[3];

		augVecM(1,0) = augVec[1];
		augVecM(1,1) = augVec[0];
		augVecM(1,2) = (-1.0)*augVec[3];
		augVecM(1,3) = augVec[2];

		augVecM(2,0) = augVec[2];
		augVecM(2,1) = augVec[3];
		augVecM(2,2) = augVec[0];
		augVecM(2,3) = (-1.0)*augVec[1];

		augVecM(3,0) = augVec[3];
		augVecM(3,1) = (-1.0)*augVec[2];
		augVecM(3,2) = augVec[1];
		augVecM(3,3) = augVec[0];

		//Apply first quaternion
		TooN::Vector<4,double> tempVec2 = augVecM*tempQuat;

		//Construct the inverse quaternion
		invQuat = TooN::makeVector(tempQuat[0],(-1)*tempQuat[1],(-1)*tempQuat[2],(-1)*tempQuat[3]);

		//Contruct inverse quaternion matrix
		augVecM(0,0) = invQuat[0];
		augVecM(0,1) = (-1.0)*invQuat[1];
		augVecM(0,2) = (-1.0)*invQuat[2];
		augVecM(0,3) = (-1.0)*invQuat[3];

		augVecM(1,0) = invQuat[1];
		augVecM(1,1) = invQuat[0];
		augVecM(1,2) = (-1.0)*invQuat[3];
		augVecM(1,3) = invQuat[2];

		augVecM(2,0) = invQuat[2];
		augVecM(2,1) = invQuat[3];
		augVecM(2,2) = invQuat[0];
		augVecM(2,3) = (-1.0)*invQuat[1];

		augVecM(3,0) = invQuat[3];
		augVecM(3,1) = (-1.0)*invQuat[2];
		augVecM(3,2) = invQuat[1];
		augVecM(3,3) = invQuat[0];


		//Apply second quaternion
		TooN::Vector<4,double> temp_out = augVecM*tempVec2;

		return TooN::makeVector(temp_out[1],temp_out[2],temp_out[3]);		
	}
}

#endif
//eof
