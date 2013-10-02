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
		
		temp_out[0] = cos(tempEuler[0])*cos(tempEuler[1])*cos(tempEuler[2]) + sin(tempEuler[0])*sin(tempEuler[1])*sin(tempEuler[2]);
		temp_out[1] = sin(tempEuler[0])*cos(tempEuler[1])*cos(tempEuler[2]) - cos(tempEuler[0])*sin(tempEuler[1])*sin(tempEuler[2]);
		temp_out[2] = cos(tempEuler[0])*sin(tempEuler[1])*cos(tempEuler[2]) + sin(tempEuler[0])*cos(tempEuler[1])*sin(tempEuler[2]);
		temp_out[3] = cos(tempEuler[0])*cos(tempEuler[1])*sin(tempEuler[2]) - sin(tempEuler[0])*sin(tempEuler[1])*cos(tempEuler[2]);
		

		//Check quaternion has unit magnitude.
		r_check_q(temp_out);

		return temp_out;
	}
}

#endif
//eof
