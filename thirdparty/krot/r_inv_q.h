#ifndef _KROT_INV_Q_H
#define _KROT_INV_Q_H

//Cross-referenced rotation library functions
#include <TooN/TooN.h>


//NOTE:: the TooN library is used and must be included in the compile search path.

namespace krot
{
	//Invert the quaternion but making the x,y,z parts negative. This is the complex conjugate.
	static void r_inv_q(TooN::Vector<4,double> &temp)
	{
		TooN::Vector<4,double> temp_out = temp;
		temp[0] = temp_out[0];
		temp[1] = (-1)*temp_out[1];
		temp[2] = (-1)*temp_out[2];
		temp[3] = (-1)*temp_out[3];
	}
}

#endif
//eof
