#ifndef _KROT_DCM_TO_E_H
#define _KROT_DCM_TO_E_H
#define PI 3.14159265359

//Cross-referenced rotation library functions
#include <r_load_tol.h>
#include <r_check_e.h>
#include <TooN/TooN.h>


//NOTE:: the TooN library is used and must be included in the compile search path.

namespace krot
{
	//Convert direction-cosine-matrix into Euler angles.
	//DCM is 3 by 3 and special orthogonal.
	//Euler triplet in [phi,theta,psi]' order in radians (the rotation order is yaw(psi),pitch(theta),roll(phi))
	inline TooN::Vector<3,double> r_dcm_to_e(TooN::Matrix<3,3,double> temp)
	{
		TooN::Vector<3,double> temp_out;

		//Find a value for theta
		temp_out[1] = asin((-1)*temp(0,2));

		//Check if theta (pitch) is close to +-90 degrees
		double tempTol = r_load_tol();
		if ( abs(cos(temp_out[1])) > tempTol ) {
		    //If not too close to +-90 then find phi and psi (roll and yaw)
		    temp_out[0] = atan2( temp(1,2) , temp(2,2) );
		    temp_out[2] = atan2( temp(0,1) , temp(0,0) );
		}
		    
		else if ( (temp(0,2)<(-1+tempTol)) && (temp(0,2)>(-1-tempTol)) ) {     
		    //If theta is at 90 then set phi=0, theta=pi/2, and calculate psi
		    temp_out[0] = 0;
		    temp_out[1] = PI/2;
		    temp_out[2] = atan2(-temp(1,0),temp(2,0));
		    std::cout<< "Gimbal lock detected. Pitch = pi/2" << std::endl;
		}
		    
		else {                      
		    //If theta is at -90 then set phi=0, theta=-pi/2, and calculate psi
		    temp_out[0] = 0;
		    temp_out[1] = -PI/2;
		    temp_out[2] = atan2(-temp(1,0),temp(2,0));
		    std::cout<< "Gimbal lock detected. Pitch = -pi/2" << std::endl;
		}
		
		//Check Euler triplet is in the correct ranges
		r_check_e(temp_out);

		return temp_out;
	}
}

#endif
//eof
