#ifndef RALGO_HELIX_LINEAR_WISHFEED_NODE_H
#define RALGO_HELIX_LINEAR_WISHFEED_NODE_H

#include <ralgo/heimer/types.h>
#include <ralgo/heimer/wishfeed_node.h>

namespace heimer 
{
	template <int Dim>
	class linear_phase_transformer : public phase_transformer_node<Dim, Dim>
	{
		linalg::mat<real,Dim> left_to_right_transform;
		linalg::mat<real,Dim> right_to_left_transform;

	public:
		linear_phase_transformer() : 
			left_to_right_transform(linalg::identity),
			right_to_left_transform(linalg::identity)
		{}

		void init(linalg::mat<real,Dim> matrix) 
		{
			right_to_left_transform = matrix;
			left_to_right_transform = inverse(matrix);
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
	};
}

#endif