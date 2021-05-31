#include <doctest/doctest.h>
#include <ralgo/heimer/datanode.h>
#include <ralgo/linalg/matrix_view.h>
#include <ralgo/heimer/command_center_2.h>

TEST_CASE("wishfeed")
{
	SUBCASE("0")
	{
		int sts;
		heimer::command_center_2 cmdcenter;
		heimer::datanode * dnode;
		heimer::node * node;

		dnode = cmdcenter.create_datanode("x0", DATANODE_TYPEHINT_SERVOWISHFEED); CHECK(dnode != nullptr);
		dnode = cmdcenter.create_datanode("y0", DATANODE_TYPEHINT_SERVOWISHFEED); CHECK(dnode != nullptr);
		dnode = cmdcenter.create_datanode("z0", DATANODE_TYPEHINT_SERVOWISHFEED); CHECK(dnode != nullptr);

		dnode = cmdcenter.create_datanode("x1", DATANODE_TYPEHINT_SERVOWISHFEED); CHECK(dnode != nullptr);
		dnode = cmdcenter.create_datanode("y1", DATANODE_TYPEHINT_SERVOWISHFEED); CHECK(dnode != nullptr);
		dnode = cmdcenter.create_datanode("z1", DATANODE_TYPEHINT_SERVOWISHFEED); CHECK(dnode != nullptr);

		heimer::real mat[9];
		ralgo::matrix_view_co<heimer::real> trans(mat, 3, 3);
		ralgo::matops::diag(trans, {1,1,1});

		node = cmdcenter.create_linear_servowf_node(
			"xyz",
			trans,
			{"x0", "y0", "z0"}, 
			{"x1", "y1", "z1"}
		);
	}
}