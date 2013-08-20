#ifndef _KROT_CHECK_Q_H
#define _KROT_CHECK_Q_H

//Cross-referenced rotation library functions
#include <TooN/TooN.h>  //TooN is required
#include <r_load_tol.h>
#include <r_check_q.h>

namespace krot
{
	//Check the quaternion is valid (i.e. unit magnitude)
	//NOTE: the quaternion is passed by reference but is NOT const. This function can change the value.
	static void r_check_q(TooN::Vector<4,double> &tempQuat)
	{
		//Check quaternion has 4 elements
		if (tempQuat.size() != 4)
		{
			std::cerr << "Quaternion does not have 4 elements." << std::endl;
		}

		//Check if the quaternion norm is close to unity.
		double tempTol = r_load_tol();
		double tempNorm = TooN::norm(tempQuat);

		std::cout << (tempNorm-1) << std::endl;

		if ( ( tempNorm > (1.0 + tempTol) ) || ( tempNorm < (1.0 - tempTol) ) )
		{
			std::cerr << "Quaternion norm is far from unity." << std::endl;
		}

		//If the norm is close to unity then re-normalise. The TooN normalize function passes by reference.
		TooN::normalize(tempQuat);
	}
}

#endif
//eof
