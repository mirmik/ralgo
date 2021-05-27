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

		int _sigdim;
		int _sigcount;

	public:
		linear_wfnode() = default;
		linear_wfnode(int sigcount, int sigdim)
		{
			_sigcount = sigcount;
			_sigdim = sigdim;
		}

		template<class M>
		void init(const M& matrix, int sigdim)
		{
			auto rl_transform = ralgo::matrix_view<real> { rl_transform_buffer, 0, 0 };
			auto lr_transform = ralgo::matrix_view<real> { lr_transform_buffer, 0, 0 };

			_sigcount = matrix.rows();
			_sigdim = sigdim;

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

		using getter_ptr = real*(wishfeed::*)();
		void process(
			igris::array_view<wishfeed *> inbuf, 
			igris::array_view<wishfeed *> outbuf, 
			getter_ptr getter, 
			real * transform_buffer
		)
		{
			real signal_buffer[_sigcount * _sigdim];
			real result_buffer[_sigcount * _sigdim];

			auto sigmat = signals_as_matrix(inbuf, getter, signal_buffer);

			ralgo::matrix_view<real> result(result_buffer, 0, 0);
			ralgo::matrix_view<real> transform { transform_buffer, _sigcount, _sigcount };

			ralgo::matops::multiply(transform, sigmat, result);

			nos::println(result);
			set_signals(result, getter, outbuf);
		}

		ralgo::matrix_view<real> signals_as_matrix(
			igris::array_view<wishfeed *> inbuf, 
			getter_ptr getter, 
			real * signal_buffer)
		{
			ralgo::matrix_view<real> signal {signal_buffer, _sigcount, _sigdim};

			for (int i = 0; i < _sigcount; ++i)
			{
				wishfeed * sig = inbuf[i];
				real * data = (sig->*getter)();
				for (int j = 0; j < _sigdim; ++j)
				{
					signal.at(i, j) = data[j];
				}
			}

			return signal;
		}

		void set_signals(
			const ralgo::matrix_view<real> & result, 
			getter_ptr getter, 
			igris::array_view<wishfeed *> outsigs)
		{
			for (int i = 0; i < _sigcount; ++i)
			{
				auto * sig = outsigs[i];
				auto * data = (sig->*getter)();
				for (int j = 0; j < _sigdim; ++j)
				{
					data[j] = result.at(i, j);
				}
			}
		}
	};
}

#endif