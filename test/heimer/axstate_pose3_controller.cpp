#include <doctest/doctest.h>
#include <ralgo/heimer/axstate_pose3_chain_processor.h>
#include <ralgo/heimer/command.h>
#include <ralgo/heimer/axis_stub_processor.h>

TEST_CASE("axstate_chain3_translation_processor") 
{
	heimer::axis_state a("a"), b("b"), c("c"), d("d"), e("e"), f("f");
	heimer::phase_signal<3> p("p");

	heimer::axis_stub_processor a_stub("a_stub");
	heimer::axis_stub_processor b_stub("b_stub");
	heimer::axis_stub_processor c_stub("c_stub");
	heimer::axis_stub_processor d_stub("d_stub");
	heimer::axis_stub_processor e_stub("e_stub");
	heimer::axis_stub_processor f_stub("f_stub");

	a_stub.bind(&a);
	b_stub.bind(&b);
	c_stub.bind(&c);
	d_stub.bind(&d);
	e_stub.bind(&e);
	f_stub.bind(&f);

	heimer::heimer_reinit();
	heimer::axstate_chain3_translation_processor posectr("posectr", 6);	
	posectr.allocate_resources();

	heimer::signal_head * posectr_left[] = { &a, &b, &c, &d, &e, &f };
	heimer::signal_head * posectr_right[] = { &p };
	posectr.set_leftside(posectr_left);
	posectr.set_rightside(posectr_right);

	posectr.set_constant(0, 10,0,0, 0,0,0);
	posectr.set_constant(1, 10,0,0, 0,0,0);
	posectr.set_constant(2, 10,0,0, 0,0,0);
	posectr.set_constant(3, 10,0,0, 0,0,0);
	posectr.set_constant(4, 10,0,0, 0,0,0);
	posectr.set_constant(5, 10,0,0, 0,0,0);
	posectr.set_constant(6, 10,0,0, 0,0,0);

	posectr.set_sensivity(0, 1,0,0, 0,0,0);
	posectr.set_sensivity(1, 0,1,0, 0,0,0);
	posectr.set_sensivity(2, 0,0,1, 0,0,0);
	posectr.set_sensivity(3, 0,0,0, 1,0,0);
	posectr.set_sensivity(4, 0,0,0, 0,1,0);
	posectr.set_sensivity(5, 0,0,0, 0,0,1);

	posectr.feedback(discrete_time());

	CHECK_EQ(p.feedpos.x, 60);
	CHECK_EQ(p.feedpos.y, 0);
	CHECK_EQ(p.feedpos.z, 0);
}