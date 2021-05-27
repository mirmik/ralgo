#include <doctest/doctest.h>
#include <ralgo/heimer/wishfeed.h>
#include <ralgo/heimer/wishfeed_node.h>
#include <ralgo/heimer/linear_wfnode.h>

TEST_CASE("wishfeed")
{
	SUBCASE("0")
	{
		heimer::wishfeed l0;
		heimer::wishfeed l1;
		heimer::wishfeed l2;

		heimer::wishfeed r0;
		heimer::wishfeed r1;
		heimer::wishfeed r2;

		heimer::linear_wfnode node;

		node.left_signals()[0] = &l0;
		node.left_signals()[1] = &l1;
		node.left_signals()[2] = &l2;
		
		node.right_signals()[0] = &r0;
		node.right_signals()[1] = &r1;
		node.right_signals()[2] = &r2;

		node.set_dim(3, 3);
		node.bind_signals({&l0,&l1,&l2}, {&r0,&r1,&r2});


		node.set_matrix(ralgo::matrix_view_ro<float>({1,0,0,0,1,0,0,0,1}, 3, 3));
	}



}