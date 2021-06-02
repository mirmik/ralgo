#ifndef RALGO_HELIX_LINEAR_WISHFEED_NODE_H
#define RALGO_HELIX_LINEAR_WISHFEED_NODE_H

#include <ralgo/heimer/types.h>
#include <ralgo/heimer/servo_wishfeed_node.h>
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
	class linear_servowf_node : public servo_wishfeed_transnode
	{
		using parent = servo_wishfeed_transnode;

		real lr_transform_buffer[16];
		real rl_transform_buffer[16];
		
	public:
		//linear_servowf_node() = default;

		template<class M>
		int init(
			const char * name, 
			const M& matrix,
			const igris::array_view<datanode_ptr> & lsigs,
			const igris::array_view<datanode_ptr> & rsigs
		) 
		{
			set_matrix(matrix);
			return servo_wishfeed_transnode::init(name, lsigs, rsigs);
		}
		
		template<class M>
		void set_matrix(const M& matrix)
		{
			auto rl_transform = ralgo::matrix_view_co<real> { rl_transform_buffer, 0, 0 };
			auto lr_transform = ralgo::matrix_view_co<real> { lr_transform_buffer, 0, 0 };

			ralgo::matops::assign(matrix, rl_transform);
			ralgo::matops::square_matrix_inverse(matrix, lr_transform);
		}

		void provide_wish_signal(real servo_wishfeed::* fieldptr) override
		{
			int sigcount = _right_dim; // == _left_dim

			real signal_buffer[sigcount];
			real result_buffer[sigcount];

			for (int i = 0; i < _right_dim; ++i) 
			{
				signal_buffer[i] = (_right_signals[i]->as_servowf()).*fieldptr;
			}

			ralgo::matrix_view_co<real> sigmat(signal_buffer, sigcount, 1);
			ralgo::matrix_view_co<real> result(result_buffer, 0, 0);
			ralgo::matrix_view_co<real> transform { rl_transform_buffer, sigcount, sigcount };
			
			ralgo::matops::multiply(transform, sigmat, result);

			for (int i = 0; i < _right_dim; ++i) 
			{
				(_left_signals[i]->as_servowf()).*fieldptr = result_buffer[i];
			}
		}

		void provide_feed_signal(real servo_wishfeed::* fieldptr) override
		{
			int sigcount = _right_dim; // == _left_dim

			real signal_buffer[sigcount];
			real result_buffer[sigcount];

			for (int i = 0; i < sigcount; ++i) 
			{
				signal_buffer[i] = (_right_signals[i]->as_servowf()).*fieldptr;
			}

			ralgo::matrix_view_co<real> sigmat(signal_buffer, sigcount, 1);
			ralgo::matrix_view_co<real> result(result_buffer, 0, 0);
			ralgo::matrix_view_co<real> transform { lr_transform_buffer, sigcount, sigcount };

			ralgo::matops::multiply(transform, sigmat, result);

			for (int i = 0; i < _right_dim; ++i) 
			{
				(_left_signals[i]->as_servowf()).*fieldptr = result_buffer[i];
			}
		}

		heimer::errcode command(int argc, const char ** argv) override
		{
			heimer::errcode sts;

			if (sts = parent::command(argc, argv)) 
				return sts; 

			return heimer::errcode::UNRESOLVED_COMMAND;
		}
	};
}

#endif