#ifndef RALGO_INTERPOLATE_H
#define RALGO_INTERPOLATE_H

namespace ralgo 
{
	static inline 
	double lerp(double strt, double fini, double k) 
	{ 
		return (1-k) * strt + k * fini; 
	}
	
	static inline 
	double lerpkoeff(double left, double right, double target) 
	{ 
		return ( target - left ) / ( right - left ); 
	}
}

#endif