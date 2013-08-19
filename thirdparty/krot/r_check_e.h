#ifndef _KROT_CHECK_E_H
#define _KROT_CHECK_E_H
#define PI 3.14159265359

//Cross-referenced rotation library functions
#include <TooN/TooN.h>
#include <math.h>


//NOTE:: the TooN library is used and must be included in the compile search path.

namespace krot
{
	//Check the Euler triplet is valid.
	//Phi and Psi are wrapped into the range (-pi:pi], Theta is wrapped into the range (-pi/2:pi/2]
	static void r_check_e(TooN::Vector<3,double> &temp)
	{
		//Check quaternion has 4 elements
		if (temp.size() != 3)
			std::cerr << "Euler triplet does not have 3 elements" << std::endl;

		//Wrap the angles to the range (-pi:pi]
		for(int aa=0 ; aa<3 ; aa++) {
			if ( temp[aa] >= 0.0 ) {
				//Remove superflous complete rotations
				temp[aa] = fmod( temp[aa] , (2*PI));
				if ( temp[aa] > PI ) {
					std::cout << "Euler angle > pi. Wrapping to (-pi:pi]." << std::endl;
					temp[aa] = temp[aa] - (2*PI);
				}
			}
			else {
				//Remove superflous complete rotations
				temp[aa] = fmod( temp[aa] , (-2*PI));
				if ( temp[aa] <= -PI ) {
					std::cout << "Euler angle <= -pi. Wrapping to (-pi:pi]." << std::endl;
					temp[aa] = temp[aa] + (2*PI);
				}
			}
		}
	}
}

#endif
//eof
