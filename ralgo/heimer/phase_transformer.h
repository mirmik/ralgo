#ifndef HEIMER_PHASE_TRANSFORMER_H
#define HEIMER_PHASE_TRANSFORMER_H

namespace ralgo 
{
	template < int LeftDim, int RightDim >
	class phase_transfomer : public wishfeed_node<phase, LeftDim, RightDim>
	{
		phase_transfomer() : wishfeed_node<phase, LeftDim, RightDim>

	};
}

#endif