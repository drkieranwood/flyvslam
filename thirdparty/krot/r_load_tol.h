#ifndef _KROT_LOAD_TOL_H
#define _KROT_LOAD_TOL_H

//Cross-referenced rotation library functions
// <none>


//NOTE:: the TooN library is used and must be included in the compile search path.

namespace krot
{
	//Load numerical tolerance for checking unit vectors etc.
	inline double r_load_tol()
	{
		return double(0.0000000001);
	}
}

#endif
//eof
