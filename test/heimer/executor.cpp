#include <doctest/doctest.h>
#include <ralgo/heimer/executor.h>
#include <ralgo/heimer/axis_state.h>
#include <ralgo/heimer/axis_stub_processor.h>
#include <ralgo/heimer/axisctr.h>
#include <ralgo/heimer/command.h>
#include <ralgo/heimer/axstate_linear_processor.h>

using namespace heimer;

std::string str(igris::buffer buf) 
{
	return std::string(buf.data(), buf.size());
}

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

	axis_stub_processor xstub("xstub");
	axis_stub_processor ystub("ystub");
	xstub.bind(&state00);
	ystub.bind(&state01);

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
	executor.append_processor(&xstub);
	executor.append_processor(&ystub);
	executor.append_processor(&axctr0);
	executor.append_processor(&linproc);

	CHECK_EQ(executor.order_table_size, 4);

	CHECK_EQ(dlist_size(&signals_list), 4);

	executor.order_sort();

	CHECK_EQ(str(executor_table[0]->name()), str(xstub.name()));
	CHECK_EQ(str(executor_table[1]->name()), str(ystub.name()));
	CHECK_EQ(str(executor_table[2]->name()), str(linproc.name()));
	CHECK_EQ(str(executor_table[3]->name()), str(axctr0.name()));
}


TEST_CASE("executor: tandem sort") 
{
	heimer_reinit();

	axis_state x("x");
	axis_state y("y");	
	axis_state vx("vx");
	axis_state vy("vy");

	axis_stub_processor xstub("xstub");
	axis_stub_processor ystub("ystub");
	xstub.bind(&x);
	ystub.bind(&y);

	axis_settings xctr_sett[1];
	axis_settings yctr_sett[1];
	axis_settings vxctr_sett[1];
	axis_settings vyctr_sett[1];

	axis_controller xctr("xctr", xctr_sett, 1);
	axis_controller yctr("yctr", yctr_sett, 1);
	axis_controller vxctr("vxctr", vxctr_sett, 1);
	axis_controller vyctr("vyctr", vyctr_sett, 1);

	axis_state * axctr_states[] = { &x, &y, &vx, &vy };

	xctr.set_controlled(&axctr_states[0]);
	yctr.set_controlled(&axctr_states[1]);
	vxctr.set_controlled(&axctr_states[2]);
	vyctr.set_controlled(&axctr_states[3]);

	axis_state * linproc_left[] =  { &x,  &y};
	axis_state * linproc_right[] = {&vx, &vy};
	float linproc_matrix[4] = { 1, 0, 0, 1 };
	float linproc_inverse_matrix[4];
	axstate_linear_processor linproc;
	linproc.init("linproc", 2, linproc_left, linproc_right, 
		linproc_matrix, linproc_inverse_matrix);

	signal_processor * executor_table[10];

	heimer::executor executor;	
	executor.set_order_table(executor_table, 10, 0);
	executor.append_processor(&xctr);
	executor.append_processor(&yctr);
	executor.append_processor(&vxctr);
	executor.append_processor(&vyctr);

	executor.append_processor(&xstub);
	executor.append_processor(&ystub);

	executor.append_processor(&linproc);
	
	CHECK_EQ(executor.order_table_size, 7);
	CHECK_EQ(dlist_size(&signals_list), 4);

	executor.order_sort();

	CHECK_EQ(str(executor_table[0]->name()), str(xstub.name()));
	CHECK_EQ(str(executor_table[1]->name()), str(xctr.name()));
	CHECK_EQ(str(executor_table[2]->name()), str(ystub.name()));
	CHECK_EQ(str(executor_table[3]->name()), str(yctr.name()));
	CHECK_EQ(str(executor_table[4]->name()), str(linproc.name()));
	CHECK_EQ(str(executor_table[5]->name()), str(vxctr.name()));
	CHECK_EQ(str(executor_table[6]->name()), str(vyctr.name()));
}