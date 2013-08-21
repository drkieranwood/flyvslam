#ifndef _KROT_APPLY_Q_H
#define _KROT_APPLY_Q_H

//Cross-referenced rotation library functions
#include <TooN/TooN.h>  //TooN is required
#include <r_inv_q.h>
#include <r_multi_q.h>

namespace krot
{	
	//Apply a quaternion rotation to a vector. This is done in the order --> v = qinv * (u * q)
	//where the * operator represents quaternion multiplication.
	inline TooN::Vector<3,double> r_apply_q(TooN::Vector<3,double> const &tempVec,TooN::Vector<4,double> const &tempQuat)
	{
		//Construct an augmented vector
		TooN::Vector<4,double> augVec = TooN::makeVector(0.0,tempVec[0],tempVec[1],tempVec[2]);

		//Quaternion multiply
		TooN::Vector<4,double> augVectwo = r_multi_q(augVec,tempQuat);
		
		//Quaternion multiply by the inverse
		TooN::Vector<4,double> invQuat = r_inv_q(tempQuat);
		augVec = r_multi_q(invQuat,augVectwo);

		//Return the last three elements of the augmented matrix.
		return TooN::makeVector(augVec[1],augVec[2],augVec[3]);	
	}
}

#endif
//eof
