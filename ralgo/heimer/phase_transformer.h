#ifndef HEIMER_PHASE_TRANSFORMER_H
#define HEIMER_PHASE_TRANSFORMER_H

namespace ralgo 
{
	template < int LeftDim, int RightDim >
	class phase_transfomer : public wishfeed_node<phase, LeftDim, RightDim>
	{
		phase_transfomer() : wishfeed_node<phase, LeftDim, RightDim>

		linalg::mat<real, LeftDim, 2> left_feed_as_matrix()
		{
			linalg::mat<real, LeftDim, 2> matrix;
			auto & signals = parent::left_signals();
			
			for (int i = 0; i < LeftDim; ++i) 
			{
				real * ptr = signals[i]._feed;

				matrix[0][i] = ptr[0];
				matrix[1][i] = ptr[1];
			}	

			return matrix;
		}
	};
}

#endif