#ifndef _KROT_CHECK_Q_H
#define _KROT_CHECK_Q_H

//Cross-referenced rotation library functions
#include <r_load_tol.h>
#include <TooN/TooN.h>


//NOTE:: the TooN library is used and must be included in the compile search path.

namespace krot
{
	//Check the quaternion is valid (i.e. unit magnitude)
	inline void r_check_q(TooN::Vector<4,double> &temp)
	{
		//Check quaternion has 4 elements
		if (temp.size() != 4)
			std::cerr << "Quaternion does not have 4 elements" << std::endl;

		//Check quatrnion norm is close to unity
		double tempTol = r_load_tol();
		if ( ( (TooN::norm(temp)-1.0) > tempTol) || ( (TooN::norm(temp)-1.0) < (-tempTol)) )
			std::cerr << "Quaternion norm is far from unity" << std::endl;

		//If the norm is close to unity then re-normalise
		TooN::normalize(temp);
	}
}

#endif
//eof
