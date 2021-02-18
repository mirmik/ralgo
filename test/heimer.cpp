#include <ralgo/heimer/control.h>
#include <ralgo/heimer/stub_axis.h>
#include <ralgo/heimer/axisctr.h>
#include <ralgo/heimer/tandem.h>
#include <ralgo/heimer/linintctr.h>

#include <ralgo/heimer/command_center.h>

#include <igris/math.h>
#include <igris/datastruct/argvc.h>

#include <main.h>

class test_control_node : public heimer::control_node
{
	std::vector<heimer::control_node*> ctrs;

public:

	test_control_node(
	    const char* name,
	    std::vector<heimer::control_node*> vec) :
		heimer::control_node(name),
		ctrs(vec) {}

	void serve_impl() override {}
	bool on_interrupt(heimer::control_node*, heimer::control_node*, heimer::interrupt_args*) override 
	{
		return true;
	}

	heimer::control_node* iterate(heimer::control_node* it) override
	{
		if (ctrs.size() == 0)
		{
			return NULL;
		}

		if (it == NULL)
		{
			return *ctrs.begin();
		}

		if (it == *(ctrs.begin() + (ctrs.size() - 1)))
		{
			return NULL;
		}

		for (int i = 0; i < ctrs.size() -1; ++i) 
		{
			if (ctrs[i] == it)
				return ctrs[i+1];
		}

		return NULL;
	}
};

LT_BEGIN_TEST(ralgo_test_suite, heimer_control_test)
{
	int sts;

	test_control_node a("a", {});
	test_control_node b("b", {&a});
	test_control_node d1("d1", {});
	test_control_node d2("d2", {});
	test_control_node d3("d3", {});
	test_control_node c("c", {&b, &d1, &d2, &d3});

	sts = b.activate();
	CHECK(sts != 0);
	CHECK(a.is_active() == false);
	CHECK(b.is_active() == false);

	sts = a.activate();
	CHECK(sts == 0);
	CHECK(a.is_active() == true);
	CHECK(b.is_active() == false);

	sts = a.deactivate();
	CHECK(sts == 0);
	CHECK(a.is_active() == false);
	CHECK(b.is_active() == false);

	sts = a.activate();
	CHECK(sts == 0);

	sts = b.activate();
	CHECK(sts == 0);
	CHECK(a.is_active() == true);
	CHECK(a.is_controlled() == true);
	CHECK(b.is_active() == true);

	sts = a.deactivate();
	CHECK(sts != 0);
	CHECK(a.is_active() == true);
	CHECK(b.is_active() == true);

	sts = d1.activate();
	CHECK_EQ(sts, 0);

	sts = c.activate();
	CHECK_EQ(sts, -4);
	CHECK(a.is_active() == true);
	CHECK(b.is_active() == true);
	CHECK(c.is_active() == false);
	CHECK(d1.is_active() == true);
	CHECK(d2.is_active() == false);
	CHECK(d3.is_active() == false);

	sts = d2.activate();
	sts = d3.activate();

	sts = c.activate();
	CHECK_EQ(sts, 0);
	CHECK(a.is_active() == true);
	CHECK(b.is_active() == true);
	CHECK(c.is_active() == true);
	CHECK(d1.is_active() == true);
	CHECK(d2.is_active() == true);
	CHECK(d3.is_active() == true);

	sts = d1.deactivate();
	CHECK_EQ(sts, -2);

	sts = a.deactivate();
	CHECK_EQ(sts, -2);

	CHECK_EQ(a.is_controlled(), true);
	CHECK_EQ(b.is_controlled(), true);
	CHECK_EQ(c.is_controlled(), false);
	CHECK_EQ(d1.is_controlled(), true);
	CHECK_EQ(d2.is_controlled(), true);
	CHECK_EQ(d3.is_controlled(), true);

	c.deactivate();
	CHECK_EQ(a.is_controlled(), true);
	CHECK_EQ(b.is_controlled(), false);
	CHECK_EQ(c.is_controlled(), false);
	CHECK_EQ(d1.is_controlled(), false);
	CHECK_EQ(d2.is_controlled(), false);
	CHECK_EQ(d3.is_controlled(), false);

	b.deactivate();
	CHECK_EQ(a.is_controlled(), false);
	CHECK_EQ(b.is_controlled(), false);
	CHECK_EQ(c.is_controlled(), false);
	CHECK_EQ(d1.is_controlled(), false);
	CHECK_EQ(d2.is_controlled(), false);
	CHECK_EQ(d3.is_controlled(), false);

	sts = a.deactivate();
	CHECK_EQ(sts, 0);
	CHECK_EQ(a.is_active(), false);
	sts = a.deactivate();	
	CHECK_EQ(sts, 0);
	CHECK_EQ(a.is_active(), false);

	sts = a.activate();
	CHECK_EQ(sts, 0);
	CHECK_EQ(a.is_active(), true);
	sts = a.activate();	
	CHECK_EQ(sts, 0);
	CHECK_EQ(a.is_active(), true);
}
LT_END_TEST(heimer_control_test)

TEST_CASE("heimer_movement_test" * doctest::timeout(2))
{
	int sts;

	heimer::stub_axis<float, float> ax("ax");
	heimer::axisctr axctr("axctr", &ax);

	heimer::set_global_protection(false);

	sts = ax.activate();
	CHECK_EQ(sts, 0);

	sts = axctr.activate();
	CHECK_EQ(sts, 0);

	axctr.set_accdcc(10, 10);
	axctr.set_speed(4);

	float target = 5;
	axctr.incmove(target);

	float minspd, minpos = std::numeric_limits<float>::max();
	float maxspd, maxpos = std::numeric_limits<float>::min();

	while(1) 
	{
		ax.feedback();
		
		axctr.serve();
		ax.serve();

		auto pos = ax.feedpos; 
		auto spd = ax.feedspd;

		if (maxspd < spd) maxspd = spd;
		if (maxpos < pos) maxpos = pos;

		if (minspd > spd) minspd = spd;
		if (minpos > pos) minpos = pos;

		if (igris::early(pos, target)) 
		{
			break;
		}
	}

	CHECK(igris::early(maxspd, 4.0));
	CHECK(igris::early(maxpos, 5.0));
}

TEST_CASE("heimer_movement_test_with_spattern" * doctest::timeout(2))
{
	int sts;

	heimer::stub_axis<float, float> ax("ax");
	heimer::axisctr axctr("axctr", &ax);
	axctr.enable_full_spattern = true;

	heimer::set_global_protection(false);

	sts = ax.activate();
	CHECK_EQ(sts, 0);

	sts = axctr.activate();
	CHECK_EQ(sts, 0);

	axctr.set_accdcc(10, 10);
	axctr.set_speed(4);

	float target = 5;
	axctr.incmove(target);

	float minspd, minpos = std::numeric_limits<float>::max();
	float maxspd, maxpos = std::numeric_limits<float>::min();

	CHECK(axctr.linear_trajectory()->spddeform.full_spattern);

	while(1) 
	{
		ax.feedback();
		
		axctr.serve();
		ax.serve();

		auto pos = ax.feedpos; 
		auto spd = ax.feedspd;

		if (maxspd < spd) maxspd = spd;
		if (maxpos < pos) maxpos = pos;

		if (minspd > spd) minspd = spd;
		if (minpos > pos) minpos = pos;

		if (igris::early(pos, target)) 
		{
			break;
		}
	}

	CHECK(igris::early(maxspd, 4.0));
	CHECK(igris::early(maxpos, 5.0));
}

LT_BEGIN_TEST(ralgo_test_suite, heimer_absmovement_test)
{
	int sts;

	heimer::stub_axis<float, float> ax("ax");
	heimer::axisctr axctr("axctr", &ax);

	sts = ax.activate();
	CHECK_EQ(sts, 0);

	sts = axctr.activate();
	CHECK_EQ(sts, 0);

	axctr.set_accdcc(10, 10);
	axctr.set_speed(10);

	float target = 5;
	axctr.absmove(target);

	float minspd, minpos = std::numeric_limits<float>::max();
	float maxspd, maxpos = std::numeric_limits<float>::min();

	while(1) 
	{
		ax.feedback();
		
		axctr.serve();
		ax.serve();

		auto pos = ax.feedpos; 
		auto spd = ax.feedspd;

		if (maxspd < spd) maxspd = spd;
		if (maxpos < pos) maxpos = pos;

		if (minspd > spd) minspd = spd;
		if (minpos > pos) minpos = pos;

		if (igris::early(pos, target)) 
		{
			break;
		}
	}

	CHECK(igris::early(maxpos, 5.0));
}
LT_END_TEST(heimer_absmovement_test)


LT_BEGIN_TEST(ralgo_test_suite, heimer_movement_stop_test)
{
	int sts;

	heimer::stub_axis<float, float> ax("ax");
	heimer::axisctr axctr("axctr", &ax);

	sts = ax.activate();
	CHECK_EQ(sts, 0);

	sts = axctr.activate();
	CHECK_EQ(sts, 0);

	axctr.set_accdcc(10, 10);
	axctr.set_speed(4);

	float target = 5;
	axctr.incmove(target);

	float minspd, minpos = std::numeric_limits<float>::max();
	float maxspd, maxpos = std::numeric_limits<float>::min();

	int state = 0;

	while(1) 
	{
		ax.feedback();
		
		axctr.serve();
		ax.serve();

		auto pos = ax.feedpos; 
		auto spd = ax.feedspd;

		if (state == 0 && igris::early(pos, 1, 0.2)) 
		{
			axctr.stop();
			state++;
		}

		if (state == 1) 
		{
			if (ax.feedspd == 0) 
				break;
		}
	}

	CHECK_LT(ax.feedpos, 5);
}
LT_END_TEST(heimer_movement_stop_test)

LT_BEGIN_TEST(ralgo_test_suite, heimer_movement_hardstop_test)
{
	int sts;

	heimer::stub_axis<float, float> ax("ax");
	heimer::axisctr axctr("axctr", &ax);

	sts = ax.activate();
	CHECK_EQ(sts, 0);

	sts = axctr.activate();
	CHECK_EQ(sts, 0);

	axctr.set_accdcc(10, 10);
	axctr.set_speed(4);

	float target = 5;
	axctr.incmove(target);

	float minspd, minpos = std::numeric_limits<float>::max();
	float maxspd, maxpos = std::numeric_limits<float>::min();

	int state = 0;

	while(1) 
	{
		ax.feedback();
		
		axctr.serve();
		ax.serve();

		auto pos = ax.feedpos; 
		auto spd = ax.feedspd;

		if (state == 0 && igris::early(pos, 1, 0.2)) 
		{
			axctr.hardstop();
			state++;
		}

		if (state == 1) 
		{
			if (ax.feedspd == 0) 
				break;
		}
	}

	CHECK_LT(ax.feedpos, 5);
}
LT_END_TEST(heimer_movement_hardstop_test)

LT_BEGIN_TEST(ralgo_test_suite, heimer_tandem)
{
	int sts;

	heimer::stub_axis<float, float> ax1("ax1");
	heimer::stub_axis<float, float> ax2("ax2");
	heimer::tandem<float, float> tand("tand", "tand.x", "tand.y", 
		&ax1, &ax2, -0.5);

	heimer::axisctr axctr1("master", &tand.master);
	heimer::axisctr axctr2("slave", &tand.slave);

	sts = ax1.activate();
	CHECK_EQ(sts, 0);
	sts = ax2.activate();
	CHECK_EQ(sts, 0);
	sts = tand.activate();
	CHECK_EQ(sts, 0);
	sts = axctr1.activate();
	CHECK_EQ(sts, 0);
	sts = axctr2.activate();
	CHECK_EQ(sts, 0);

	axctr1.set_speed(10);
	axctr2.set_speed(10);
	axctr1.set_accdcc(10,10);
	axctr2.set_accdcc(10,10);

	sts = axctr1.incmove(8);
	CHECK_EQ(sts, 0);
	sts = axctr2.incmove(8);
	CHECK_EQ(sts, 0);

	while(1) 
	{
		ax1.feedback();
		ax2.feedback();
		tand.feedback();

		axctr1.serve();
		axctr2.serve();
		tand.serve();
		ax2.serve();
		ax1.serve();

		if (!axctr1.in_operate() && !axctr2.in_operate()) 
			break;
	}

	CHECK(igris::early(8, ax1.feedpos, 1e-3));
	CHECK(igris::early(4, ax2.feedpos, 1e-3));
}
LT_END_TEST(heimer_tandem)

LT_BEGIN_TEST(ralgo_test_suite, heimer_linint)
{
	int sts;

	heimer::stub_axis<float, float> ax1("ax1");
	heimer::stub_axis<float, float> ax2("ax2");

	std::vector<heimer::axis_node<float, float> *> intarr {&ax1, &ax2};
	heimer::linintctr<float, float, 2> linint("int", {intarr.data(), 2});

	sts=ax1.activate();;
	CHECK_EQ(sts, 0);
	sts=ax2.activate();;
	CHECK_EQ(sts, 0);
	sts=linint.activate();;
	CHECK_EQ(sts, 0);

	linint.set_speed(10);
	linint.set_accdcc(100000, 100000);

	float target[2] = {10, 3};
	sts=linint.incmove(target);
	CHECK_EQ(sts, 0);		

	CHECK_EQ(linint.in_operate(), true);		

	int64_t t = ralgo::discrete_time();

	while(1) 
	{
		ax1.feedback();
		ax2.feedback();
		linint.feedback();

		linint.serve();
		ax2.serve();
		ax1.serve();

		if (!linint.in_operate()) 
			break;
	}

	CHECK(ralgo::discrete_time() - t <= 1045);
	CHECK(igris::early(10, ax1.feedpos, 1e-3));
	CHECK(igris::early(3, ax2.feedpos, 1e-3));
}
LT_END_TEST(heimer_linint)


TEST_CASE("heimer_control_panel") 
{
	heimer::stub_axis<float, float> ax1("ax1");
	heimer::stub_axis<float, float> ax2("ax2");
	heimer::axisctr axctr1("axctr1", &ax1);
	heimer::axisctr axctr2("axctr2", &ax2);

	heimer::axisctr<float,float>* arr[] = { &axctr1, &axctr2 };

	heimer::command_center.attach_axes(arr);

	ax1.activate();
	ax2.activate();
	axctr1.activate();
	axctr2.activate();

	CHECK_EQ(ax1.is_active(), true);
	CHECK_EQ(ax2.is_active(), true);
	CHECK_EQ(axctr1.is_active(), true);
	CHECK_EQ(axctr2.is_active(), true);

	int argc;
	char*argv [10];

	std::string str;

	str = std::string("axcmd 0 setspd 10");
	argc = argvc_internal_split_n(str.data(), str.size(), argv, 10);
	heimer::command_center.axcmd(argc, argv);

	str = std::string("axcmd 0 setacc 10");
	argc = argvc_internal_split_n(str.data(), str.size(), argv, 10);
	heimer::command_center.axcmd(argc, argv);

	str = std::string("axcmd 0 incmov 10");
	argc = argvc_internal_split_n(str.data(), str.size(), argv, 10);
	heimer::command_center.axcmd(argc, argv);

	while(1) 
	{
		ax1.feedback();
		ax2.feedback();

		axctr1.serve();
		axctr2.serve();
		ax2.serve();
		ax1.serve();

		if (!axctr1.in_operate()) 
			break;
	}

	CHECK(igris::early(10, ax1.ctrpos));
}
