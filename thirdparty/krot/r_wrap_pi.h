#ifndef _KROT_WRAP_PI_H
#define _KROT_WRAP_PI_H
#define _KROT_WRAP_PI_H_PI 3.14159265358979323846

//Cross-referenced rotation library functions
#include <cmath>

namespace krot
{
	//Wrap the given angle into the range (-PI:PI]
	//NOTE: passed by reference and NOT const.
	inline void r_wrap_pi(double &tempAng)
	{
		if (tempAng>=0.0) {
			//Remove superflous complete positive rotations
			tempAng = fmod( tempAng , (2*_KROT_WRAP_PI_H_PI) );
			if ( tempAng > _KROT_WRAP_PI_H_PI ) {
				tempAng = tempAng - (2*_KROT_WRAP_PI_H_PI);
			}
		}
		else {
			//Remove superflous complete negative rotations
			tempAng = fmod( tempAng , ( (-2)*_KROT_WRAP_PI_H_PI ) );
			if ( tempAng <= (-1)*_KROT_WRAP_PI_H_PI ) {
				tempAng = tempAng + (2*_KROT_WRAP_PI_H_PI);
			}
		}
	}
}

#endif
//eof
