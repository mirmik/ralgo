#ifndef RALGO_INTERPOLATE_H
#define RALGO_INTERPOLATE_H

namespace ralgo 
{
	double lerp(double strt, double fini, double k) { return (1-k) * strt + k * fini; }
}

#endif