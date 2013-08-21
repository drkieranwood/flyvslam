#ifndef _KROT_Q_TO_E_H
#define _KROT_Q_TO_E_H

//Cross-referenced rotation library functions
#include <TooN/TooN.h>  //TooN is required
#include <r_dcm_to_e.h>
#include <r_q_to_dcm.h>

namespace krot
{
	//Convert quaternion into an Euler angle triplet.
	//Quaternion in [w,x,y,z]' order with unit magnitude.
	//Euler triplet in [phi,theta,psi]' order in radians (the rotation order is yaw(psi),pitch(theta),roll(phi))
	inline TooN::Vector<3,double> r_q_to_e(TooN::Vector<4,double> const &tempQuat)
	{
		//Convert to Euler angles via an intermediate DCM
		TooN::Matrix<3,3,double> tempDCM = r_q_to_dcm(tempQuat);
		return (r_dcm_to_e(tempDCM));
	}
}

#endif
//eof
