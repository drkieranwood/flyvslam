#ifndef _KROT_CHECK_E_H
#define _KROT_CHECK_E_H

//Cross-referenced rotation library functions
#include <TooN/TooN.h>  //TooN is required
#include <r_wrap_pi.h>
#include <cmath>

namespace krot
{
	//Check the Euler triplet is valid.
	//Phi and Psi are wrapped into the range (-pi:pi], Theta is wrapped into the range (-pi/2:pi/2]
	//NOTE: the triplet is passed by reference but is NOT const. This function can change the value.
	static void r_check_e(TooN::Vector<3,double> &tempEul)
	{
		//Check the triplet has three elements
		if (tempEul.size() != 3)
			std::cerr << "Euler triplet does not have 3 elements." << std::endl;

		//Wrap the angles to the range (-pi:pi]
		for(int aa=0 ; aa<3 ; aa++) {
			r_wrap_pi( tempEul[aa] );
		}
	}
}

#endif
//eof
