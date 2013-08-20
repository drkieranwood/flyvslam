#ifndef _KROT_LOAD_TOL_H
#define _KROT_LOAD_TOL_H

//Cross-referenced rotation library functions
// <none>

namespace krot
{
	//Load numerical tolerance for checking unit vectors etc.
	inline double r_load_tol(void)
	{
		return double(0.0000001);
	}
}

#endif
//eof
