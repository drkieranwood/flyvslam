#ifndef _KROT_Q_TO_DCM_H
#define _KROT_Q_TO_DCM_H

//Cross-referenced rotation library functions
#include <r_check_q.h>
#include <TooN/TooN.h>


//NOTE:: the TooN library is used and must be included in the compile search path.

namespace krot
{
	//Convert quaternion into a direction-cosine-matrix (DCM).
	//Quaternion in [w,x,y,z]' order with unit magnitude.
	//DCM is 3 by 3 and special orthogonal.
	static TooN::Matrix<3,3,double> r_q_to_dcm(TooN::Vector<4,double> temp)
	{
		//Check the quaternion is valid.
		krot::r_check_q(temp);
	
		//Convert to DCM.
		TooN::Matrix<3,3,double> temp_out;
		temp_out(0,0) = (temp[0]*temp[0]) + (temp[1]*temp[1]) - (temp[2]*temp[2]) - (temp[3]*temp[3]);
		temp_out(0,1) = 2*( temp[1]*temp[2] + temp[0]*temp[3] );
		temp_out(0,2) = 2*( temp[1]*temp[3] - temp[0]*temp[2] );

		temp_out(1,0) = 2*( temp[1]*temp[2] - temp[0]*temp[3] );
		temp_out(1,1) = (temp[0]*temp[0]) - (temp[1]*temp[1]) + (temp[2]*temp[2]) - (temp[3]*temp[3]);
		temp_out(1,2) = 2*( temp[2]*temp[3] + temp[0]*temp[1] );

		temp_out(2,0) = 2*( temp[1]*temp[3] + temp[0]*temp[2] );
		temp_out(2,1) = 2*( temp[2]*temp[3] - temp[0]*temp[1] );
		temp_out(2,2) = (temp[0]*temp[0]) - (temp[1]*temp[1]) - (temp[2]*temp[2]) + (temp[3]*temp[3]);

		return temp_out;
	}
}

#endif
//eof
