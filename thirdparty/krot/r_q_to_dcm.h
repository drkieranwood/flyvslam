#ifndef _KROT_Q_TO_DCM_H
#define _KROT_Q_TO_DCM_H

//Cross-referenced rotation library functions
#include <TooN/TooN.h>  //TooN is required
#include <r_check_q.h>

namespace krot
{
	//Convert quaternion into a direction-cosine-matrix (DCM).
	//Quaternion in [w,x,y,z]' order with unit magnitude.
	//DCM is 3 by 3 and special orthogonal.
	//NOTE: arguments are passed by reference and NOT const because the r_check_q() function needs to be able to normalise them.
	static TooN::Matrix<3,3,double> r_q_to_dcm(TooN::Vector<4,double> &tempQuat)
	{
		//Check the quaternion is valid.
		r_check_q(tempQuat);
	
		//Convert to DCM.
		TooN::Matrix<3,3,double> temp_out;
		temp_out(0,0) = (tempQuat[0]*tempQuat[0]) + (tempQuat[1]*tempQuat[1]) - (tempQuat[2]*tempQuat[2]) - (tempQuat[3]*tempQuat[3]);
		temp_out(0,1) = 2*( tempQuat[1]*tempQuat[2] + tempQuat[0]*tempQuat[3] );
		temp_out(0,2) = 2*( tempQuat[1]*tempQuat[3] - tempQuat[0]*tempQuat[2] );

		temp_out(1,0) = 2*( tempQuat[1]*tempQuat[2] - tempQuat[0]*tempQuat[3] );
		temp_out(1,1) = (tempQuat[0]*tempQuat[0]) - (tempQuat[1]*tempQuat[1]) + (tempQuat[2]*tempQuat[2]) - (tempQuat[3]*tempQuat[3]);
		temp_out(1,2) = 2*( tempQuat[2]*tempQuat[3] + tempQuat[0]*tempQuat[1] );

		temp_out(2,0) = 2*( tempQuat[1]*tempQuat[3] + tempQuat[0]*tempQuat[2] );
		temp_out(2,1) = 2*( tempQuat[2]*tempQuat[3] - tempQuat[0]*tempQuat[1] );
		temp_out(2,2) = (tempQuat[0]*tempQuat[0]) - (tempQuat[1]*tempQuat[1]) - (tempQuat[2]*tempQuat[2]) + (tempQuat[3]*tempQuat[3]);

		return temp_out;
	}
}

#endif
//eof
