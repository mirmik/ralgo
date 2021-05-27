#ifndef RALGO_HELIX_LINEAR_WISHFEED_NODE_H
#define RALGO_HELIX_LINEAR_WISHFEED_NODE_H

#include <ralgo/heimer/types.h>
#include <ralgo/heimer/wishfeed_node.h>
#include <ralgo/linalg/matops.h>
#include <ralgo/linalg/matrix_view.h>

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

		ralgo::matrix_view<real> lr_transform;
		ralgo::matrix_view<real> rl_transform;

		int _sigdim;
		int _sigcount;

	public:
		linear_wfnode() = default;
		linear_wfnode(int sigcount, int sigdim)
		{
			_sigcount = sigcount;
			_sigdim = sigdim;
		}

		void init(int sigcount, int sigdim)
		{
			lr_transform = ralgo::matrix_view<real>(lr_transform_buffer,
			                                        sigcount, sigcount);

			rl_transform = ralgo::matrix_view<real>(lr_transform_buffer,
			                                        sigcount, sigcount);
		}

		template<class M>
		void set_matrix(const M& matrix)
		{
			rl_transform = { rl_transform_buffer, 0, 0 };
			lr_transform = { lr_transform_buffer, 0, 0 };

			ralgo::matops::assign(matrix, rl_transform);
			ralgo::matops::square_matrix_inverse(matrix, lr_transform);
		}

		void serve_feed()
		{
			real signal_buffer[_sigcount * _sigdim];
			real result_buffer[_sigcount * _sigdim];

			auto sigmat = left_feed_as_matrix(signal_buffer);
			ralgo::matrix_view<real> result(result_buffer, 0, 0);

			ralgo::matops::multiply(lr_transform, sigmat, result);
			set_right_feed(result);
		}

		void serve_wish()
		{
			//auto sigmat = right_wish_as_matrix(signal_buffer);
			//parent::set_left_wish(mul(right_to_left_transform, feed));
		}

		ralgo::matrix_view<real> left_feed_as_matrix(real * signal_buffer)
		{
			ralgo::matrix_view<real> signal(signal_buffer, _sigcount, _sigdim);
			
			for (int i = 0; i < _sigcount; ++i) 
			{
				auto & sig = *_left_signals[i];
				auto * feed = sig.feed();
				for (int j = 0; j < _sigdim; ++j)
				{
					signal.at(i,j) = feed[j];
				}
			}

			return signal;
		}

		void set_right_feed(const ralgo::matrix_view<real> & result) 
		{
			for (int i = 0; i < _sigcount; ++i) 
			{
				auto & sig = *_right_signals[i];
				auto * feed = sig.feed();
				for (int j = 0; j < _sigdim; ++j)
				{
					feed[j] = result.at(i,j);
				}
			}	
		}
	};
}

#endif