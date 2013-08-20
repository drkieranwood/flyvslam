#ifndef _KROT_Q_TO_E_H
#define _KROT_Q_TO_E_H

//Cross-referenced rotation library functions
#include <TooN/TooN.h>  //TooN is required
#include <r_check_q.h>
#include <r_check_e.h>
#include <r_dcm_to_e.h>
#include <r_q_to_dcm.h>

namespace krot
{
	//Convert quaternion into an Euler angle triplet.
	//Quaternion in [w,x,y,z]' order with unit magnitude.
	//Euler triplet in [phi,theta,psi]' order in radians (the rotation order is yaw(psi),pitch(theta),roll(phi))
	//NOTE: arguments are passed by reference and NOT const because the r_check_q() function needs to be able to normalise them.
	static TooN::Vector<3,double> r_q_to_e(TooN::Vector<4,double> &tempQuat)
	{
		//Check the quaternion is valid.
		r_check_q(tempQuat);

		//Convert to Euler angles via an intermediate DCM
		TooN::Matrix<3,3,double> tempDCM = r_q_to_dcm(tempQuat);
		TooN::Vector<3,double> temp_out = r_dcm_to_e(tempDCM);
		
		//Check Euler triplet is in the correct ranges
		r_check_e(temp_out);
		
		return temp_out;
	}
}

#endif
//eof
