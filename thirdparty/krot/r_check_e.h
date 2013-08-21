#ifndef _KROT_CHECK_E_H
#define _KROT_CHECK_E_H

//Cross-referenced rotation library functions
#include <TooN/TooN.h>  //TooN is required
#include <r_wrap_pi.h>
#include <cmath>

namespace krot
{
	//Check the Euler triplet is valid and wrap angles to the range (-pi:pi].
	//NOTE: passed by reference and NOT const.
	inline void r_check_e(TooN::Vector<3,double> &tempEul)
	{
		//Check the triplet has three elements.
		if (tempEul.size() != 3)
		{
			std::cerr << "Euler triplet does not have 3 elements." << std::endl;
		}

		//Wrap the angles to the range (-pi:pi]
		for(int aa=0 ; aa<3 ; aa++) 
		{
			r_wrap_pi( tempEul[aa] );
		}
	}
}

#endif
//eof
