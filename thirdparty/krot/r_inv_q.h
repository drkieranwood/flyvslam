#ifndef _KROT_INV_Q_H
#define _KROT_INV_Q_H

//Cross-referenced rotation library functions
#include <TooN/TooN.h>  //TooN is required

namespace krot
{
	//Invert the quaternion by making the x,y,z parts negative. This is the complex conjugate.
	static TooN::Vector<4,double> r_inv_q(TooN::Vector<4,double> const &tempQuat)
	{
		TooN::Vector<4,double> temp_out = TooN::makeVector(tempQuat[0],(-1)*tempQuat[1],(-1)*tempQuat[2],(-1)*tempQuat[3]);
		return temp_out;
	}
}

#endif
//eof
