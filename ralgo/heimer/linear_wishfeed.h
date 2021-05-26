#ifndef RALGO_HELIX_LINEAR_WISHFEED_NODE_H
#define RALGO_HELIX_LINEAR_WISHFEED_NODE_H

#include <ralgo/heimer/types.h>
#include <ralgo/heimer/wishfeed_node.h>

namespace heimer 
{
	class linear_wishfeed_transform : public wishfeed_node
	{
		real * left_to_right_transform = nullptr;
		real * right_to_left_transform = nullptr;

	public:
		void allocate_

		linear_phase_transformer() : 
			left_to_right_transform(linalg::identity),
			right_to_left_transform(linalg::identity)
		{}

		int transform_matrix_bytesize() 
		{
			return sizeof(real) * signal_count * signal_count;
		}

		int signal_matrix_bytesize() 
		{
			return sizeof(real) * signal_count * signal_dim;
		}

		void init(int signal_count, int signal_dim) 
		{
			int bytesize = transform_matrix_bytesize();

			right_to_left_transform = malloc(bytesize);
			left_to_right_transform = malloc(bytesize);
			memset(right_to_left_transform, 0, bytesize);
			memset(left_to_right_transform, 0, bytesize);
		}

		void release() 
		{
			free(left_to_right_transform);
			free(right_to_left_transform);
		} 

		void serve_feed() 
		{
			auto feed = parent::left_feed_as_matrix();
			parent::set_right_feed(mul(left_to_right_transform, feed));
		}

		void serve_wish()
		{
			auto wish = parent::right_wish_as_matrix();
			parent::set_left_wish(mul(right_to_left_transform, feed));
		}


		linalg::mat<real, LeftDim, 2> left_feed_as_matrix()
		{
			real [];
		}
	};
}

#endif