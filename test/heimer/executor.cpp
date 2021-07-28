#include <doctest/doctest.h>
#include <ralgo/heimer/executor.h>
#include <ralgo/heimer/axis_state.h>
#include <ralgo/heimer/axisctr.h>
#include <ralgo/heimer/command.h>
#include <ralgo/heimer/axstate_linear_processor.h>

using namespace heimer;

TEST_CASE("executor") 
{
	heimer_reinit();

	axis_state state00;
	axis_state state01;
	
	axis_state state10;
	axis_state state11;

	axis_controller axctr0;

	state00.init("state00");
	state01.init("state01");
	state10.init("state10");
	state11.init("state11");

	axis_state * axctr0_states[] = { &state10, &state11 };
	axis_settings axctr0_settings[2];
	axctr0.init("axctr0", axctr0_settings, 2);
	axctr0.set_controlled(axctr0_states);

	axis_state * linproc_left[] = {&state00, &state01};
	axis_state * linproc_right[] = {&state10, &state11};
	float linproc_matrix[4] = { 1, 0, 0, 1 };
	float linproc_inverse_matrix[4];
	axstate_linear_processor linproc;
	linproc.init("linproc", 2, linproc_left, linproc_right, 
		linproc_matrix, linproc_inverse_matrix);

	signal_processor * executor_table[10];

	heimer::executor executor;	
	executor.set_order_table(executor_table, 10, 0);
	executor.append_processor(&axctr0);
	executor.append_processor(&linproc);

	CHECK_EQ(executor.order_table_size, 2);

	CHECK_EQ(dlist_size(&signals_list), 4);

	executor.order_sort();

	CHECK_EQ(executor_table[0], &linproc);	
	CHECK_EQ(executor_table[1], &axctr0);
}