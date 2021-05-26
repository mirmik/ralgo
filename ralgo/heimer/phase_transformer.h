#ifndef HEIMER_PHASE_TRANSFORMER_H
#define HEIMER_PHASE_TRANSFORMER_H

namespace ralgo 
{
	template < int LeftDim, int RightDim >
	class phase_transfomer : public wishfeed_node<phase, LeftDim, RightDim>
	{
		phase_transfomer() : wishfeed_node<phase, LeftDim, RightDim>

		linalg::mat<real, LeftDim, 2> left_phase_as_matrix()
		{
			linalg::mat<real, LeftDim, 2> matrix;
			auto & signals = parent::left_signals();
			
			for (int i = 0; i < LeftDim; ++i) 
			{
				matrix[0][i] = signals[i]->pos;
				matrix[1][i] = signals[i]->pos;
			}	

			return matrix;
		}
	};
}

#endif