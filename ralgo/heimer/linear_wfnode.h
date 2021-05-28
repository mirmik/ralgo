#ifndef RALGO_HELIX_LINEAR_WISHFEED_NODE_H
#define RALGO_HELIX_LINEAR_WISHFEED_NODE_H

#include <ralgo/heimer/types.h>
#include <ralgo/heimer/wishfeed_node.h>

/*
Example :

	For signal matrix 3x2

    ------------------> sigdim
   |  | x_pos x_spd |
   |  | y_pos y_spd |
   |  | z_pos z_spd |
   v
sigcount

	need transformation matrix 3x3:
   | -----------------> sigcount
   |  | a00 a01 a02 |
   |  | a10 a11 a12 |
   |  | a20 a21 a22 |
   v
sigcount
*/

namespace heimer
{
	class linear_wfnode : public wishfeed_node
	{
		real lr_transform_buffer[16];
		real rl_transform_buffer[16];
		
	public:
		linear_wfnode() = default;

		template<class M>
		linear_wfnode(const M& matrix) 
		{
			init(matrix);
		}
		
		template<class M>
		void init(const M& matrix)
		{
			auto rl_transform = ralgo::matrix_view<real> { rl_transform_buffer, 0, 0 };
			auto lr_transform = ralgo::matrix_view<real> { lr_transform_buffer, 0, 0 };

			ralgo::matops::assign(matrix, rl_transform);
			ralgo::matops::square_matrix_inverse(matrix, lr_transform);
		}

		void do_wish_transform(
			real * signal_buffer,
			real * result_buffer,
			int sigcount,
			int sigdim
		) override
		{
			ralgo::matrix_view_co<real> sigmat(signal_buffer, sigcount, sigdim);
			ralgo::matrix_view_co<real> result(result_buffer, 0, 0);
			ralgo::matrix_view_co<real> transform { rl_transform_buffer, sigcount, sigcount };
			
			ralgo::matops::multiply(transform, sigmat, result);
		}

		void do_feed_transform(
			real * signal_buffer,
			real * result_buffer,
			int sigcount,
			int sigdim
		) override
		{
			ralgo::matrix_view_co<real> sigmat(signal_buffer, sigcount, sigdim);
			ralgo::matrix_view_co<real> result(result_buffer, 0, 0);
			ralgo::matrix_view_co<real> transform { lr_transform_buffer, sigcount, sigcount };

			ralgo::matops::multiply(transform, sigmat, result);
		}
	};
}

#endif