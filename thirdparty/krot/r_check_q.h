#ifndef _KROT_CHECK_Q_H
#define _KROT_CHECK_Q_H

//Cross-referenced rotation library functions
#include <TooN/TooN.h>  //TooN is required
#include <r_load_tol.h>
#include <r_check_q.h>

namespace krot
{
	//Check the quaternion is valid and has unit magnitude.
	//NOTE: passed by reference and NOT const.
	inline void r_check_q(TooN::Vector<4,double> &tempQuat)
	{
		//Check quaternion has 4 elements
		if (tempQuat.size() != 4)
		{
			std::cerr << "Quaternion does not have 4 elements." << std::endl;
		}

		//Check if the quaternion norm is close to unity.
		double tempTol = r_load_tol();
		double tempNorm = ( TooN::norm(tempQuat) ) - (1.0);

		if ( ( tempNorm > tempTol ) || ( tempNorm < (-1)*tempTol ) )
		{
			std::cerr << "Quaternion norm is far from unity." << std::endl;
		}

		//If the norm is close to unity then re-normalise. The TooN normalize function passes by reference.
		TooN::normalize(tempQuat);
	}
}

#endif
//eof
