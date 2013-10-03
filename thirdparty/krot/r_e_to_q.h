#ifndef _KROT_E_TO_Q_H
#define _KROT_E_TO_Q_H

//Cross-referenced rotation library functions
#include <TooN/TooN.h>  //TooN is required
#include <r_check_q.h>
#include <cmath>

namespace krot
{
	//Convert an Euler triplet into a quaternion.
	//Euler triplet in [phi,theta,psi]' order in radians (the rotation order is yaw(psi),pitch(theta),roll(phi))
	//Quaternion in [w,x,y,z]' order with unit magnitude.
	inline TooN::Vector<4,double> r_e_to_q(TooN::Vector<3,double> const &tempEuler)
	{	
		//Convert to quaternion.
		TooN::Vector<4,double> temp_out;
		
		double s1 = sin(0.5*tempEuler[0]);
		double s2 = sin(0.5*tempEuler[1]);
		double s3 = sin(0.5*tempEuler[2]);
		double c1 = cos(0.5*tempEuler[0]);
		double c2 = cos(0.5*tempEuler[1]);
		double c3 = cos(0.5*tempEuler[2]);
		
		temp_out[0] = c1*c2*c3 + s1*s2*s3;
		temp_out[1] = s1*c2*c3 - c1*s2*s3;
		temp_out[2] = c1*s2*c3 + s1*c2*s3;
		temp_out[3] = c1*c2*s3 - s1*s2*s3;
		
		//Check quaternion has unit magnitude.
		r_check_q(temp_out);

		return temp_out;
	}
}

#endif
//eof
