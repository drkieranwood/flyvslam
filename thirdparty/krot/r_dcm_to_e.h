#ifndef _KROT_DCM_TO_E_H
#define _KROT_DCM_TO_E_H
#define _KROT_DCM_TO_E_H_PI 3.14159265358979323846

//Cross-referenced rotation library functions
#include <TooN/TooN.h>  //TooN is required
#include <r_load_tol.h>
#include <r_check_e.h>
#include <cmath>

namespace krot
{
	//Convert direction-cosine-matrix into Euler angles.
	//DCM is 3 by 3 and special orthogonal.
	//Euler triplet in [phi,theta,psi]' order in radians (the rotation order is yaw(psi),pitch(theta),roll(phi))
	inline TooN::Vector<3,double> r_dcm_to_e(TooN::Matrix<3,3,double> const &tempDCM)
	{		
		TooN::Vector<3,double> temp_out;

		//Find a value for theta
		temp_out[1] = asin( (-1)*tempDCM(0,2) );

		//Check if theta (pitch) is close to +-90 degrees
		double tempTol = r_load_tol();		
		if ( fabs(cos(temp_out[1])) > tempTol ) {
		    //If not too close to +-90 (i.e. cos(theta)=0) then find phi and psi (roll and yaw)
		    temp_out[0] = atan2( tempDCM(1,2) , tempDCM(2,2) );
		    temp_out[2] = atan2( tempDCM(0,1) , tempDCM(0,0) );
		}
		else if ( (tempDCM(0,2)<(-1.0+tempTol)) && (tempDCM(0,2)>(-1.0-tempTol)) ) {     
		    //If theta is at 90 then set phi=0, theta=pi/2, and calculate psi
		    temp_out[0] = 0;
		    temp_out[1] = _KROT_DCM_TO_E_H_PI/2;
		    temp_out[2] = atan2(-tempDCM(1,0),tempDCM(2,0));
		    std::cout<< "Gimbal lock detected. Pitch = pi/2." << std::endl;
		}
		else {                      
		    //If theta is at -90 then set phi=0, theta=-pi/2, and calculate psi
		    temp_out[0] = 0;
		    temp_out[1] = ((-1)*_KROT_DCM_TO_E_H_PI)/2;
		    temp_out[2] = atan2(-tempDCM(1,0),tempDCM(2,0));
		    std::cout<< "Gimbal lock detected. Pitch = -pi/2." << std::endl;
		}
		
		//Check Euler triplet is in the correct ranges.
		r_check_e(temp_out);

		return temp_out;
	}
}

#endif
//eof
