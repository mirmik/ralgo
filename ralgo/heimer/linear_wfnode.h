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
		void init(const M& matrix, int sigdim)
		{
			auto rl_transform = ralgo::matrix_view<real> { rl_transform_buffer, 0, 0 };
			auto lr_transform = ralgo::matrix_view<real> { lr_transform_buffer, 0, 0 };

			ralgo::matops::assign(matrix, rl_transform);
			ralgo::matops::square_matrix_inverse(matrix, lr_transform);
		}

		void serve_feed()
		{
			process(
			    left_signals(),
			    right_signals(),
			    &wishfeed::feed,
			    lr_transform_buffer);
		}

		void serve_wish()
		{
			process(
			    right_signals(),
			    left_signals(),
			    &wishfeed::wish,
			    rl_transform_buffer);
		}

		void process(
			igris::array_view<wishfeed *> inbuf, 
			igris::array_view<wishfeed *> outbuf, 
			getter_ptr getter, 
			real * transform_buffer
		)
		{
			int dim = inbuf[0]->size();
			int sigs = inbuf.size();

			real signal_buffer[sigs * dim];
			real result_buffer[sigs * dim];

			auto sigmat = signals_as_matrix(inbuf, getter, signal_buffer);

			ralgo::matrix_view<real> result(result_buffer, 0, 0);
			ralgo::matrix_view<real> transform { transform_buffer, sigs, sigs };

			ralgo::matops::multiply(transform, sigmat, result);

			set_signals(result, getter, outbuf);
		}
	};
}

#endif