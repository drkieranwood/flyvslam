#ifndef _KROT_Q_TO_E_H
#define _KROT_Q_TO_E_H

//Cross-referenced rotation library functions
#include <r_check_q.h>
#include <r_check_e.h>
#include <r_dcm_to_e.h>
#include <r_q_to_dcm.h>
#include <TooN/TooN.h>


//NOTE:: the TooN library is used and must be included in the compile search path.

namespace krot
{
	//Convert quaternion into an Euler angle triplet.
	//Quaternion in [w,x,y,z]' order with unit magnitude.
	//Euler triplet in [phi,theta,psi]' order in radians (the rotation order is yaw(psi),pitch(theta),roll(phi))
	static TooN::Vector<3,double> r_q_to_e(TooN::Vector<4,double> temp)
	{
		//Check the quaternion is valid.
		krot::r_check_q(temp);

		//Convert to Euler angles via an intermediate DCM
		TooN::Vector<3,double> temp_out = r_dcm_to_e(r_q_to_dcm(temp));
		
		//Check Euler triplet is in the correct ranges
		r_check_e(temp_out);
		
		return temp_out;
	}
}

#endif
//eof
