#ifndef _KROT_WRAP_PI_H
#define _KROT_WRAP_PI_H
#define PI 3.14159265359

//Cross-referenced rotation library functions
#include <math.h>


//NOTE:: the TooN library is used and must be included in the compile search path.

namespace krot
{
	//Wrap the given angle into the range (-PI:PI]
	static void r_wrap_pi(double &temp)
	{
		if (temp>=0.0) {
			//Remove superflous complete rotations
			temp = fmod( temp , (2*PI) );
			if ( temp > PI ) {
				temp = temp - (2*PI);
			}
		}
		else {
			//Remove superflous complete rotations
			temp = fmod( temp , (-2*PI) );
			if ( temp <= -PI ) {
				temp = temp + (2*PI);
			}
		}
	}
}

#endif
//eof
