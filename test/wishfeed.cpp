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


		node.init(ralgo::matrix_view_ro<float>({1,0,0,0,2,0,0,0,1}, 3, 3), 2);

		for (auto * l : node.left_signals()) 
		{
			l->feed()[0] = 1;
			l->feed()[1] = 2;
		}

		node.serve_feed();

		CHECK_EQ(r0.feed()[0], 1);
		CHECK_EQ(r0.feed()[1], 2);
		CHECK_EQ(r1.feed()[0], 0.5);
		CHECK_EQ(r1.feed()[1], 1);
		CHECK_EQ(r2.feed()[0], 1);
		CHECK_EQ(r2.feed()[1], 2);
	}
}