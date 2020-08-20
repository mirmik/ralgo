#include <ralgo/heimer/control.h>
#include <ralgo/heimer/stub_axis.h>
#include <ralgo/heimer/axisctr.h>
#include <ralgo/heimer/tandem.h>
#include <ralgo/heimer/linintctr.h>

#include <ralgo/heimer/command_center.h>

#include <igris/math.h>
#include <igris/datastruct/argvc.h>

class test_control_node : public heimer::control_node
{
	std::vector<heimer::control_node*> ctrs;

public:

	test_control_node(
	    const char* name,
	    std::vector<heimer::control_node*> vec) :
		heimer::control_node(name),
		ctrs(vec) {}

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
	LT_CHECK(sts != 0);
	LT_CHECK(a.is_active() == false);
	LT_CHECK(b.is_active() == false);

	sts = a.activate();
	LT_CHECK(sts == 0);
	LT_CHECK(a.is_active() == true);
	LT_CHECK(b.is_active() == false);

	sts = a.deactivate();
	LT_CHECK(sts == 0);
	LT_CHECK(a.is_active() == false);
	LT_CHECK(b.is_active() == false);

	sts = a.activate();
	LT_CHECK(sts == 0);

	sts = b.activate();
	LT_CHECK(sts == 0);
	LT_CHECK(a.is_active() == true);
	LT_CHECK(a.is_controlled() == true);
	LT_CHECK(b.is_active() == true);

	sts = a.deactivate();
	LT_CHECK(sts != 0);
	LT_CHECK(a.is_active() == true);
	LT_CHECK(b.is_active() == true);

	sts = d1.activate();
	LT_CHECK_EQ(sts, 0);

	sts = c.activate();
	LT_CHECK_EQ(sts, -4);
	LT_CHECK(a.is_active() == true);
	LT_CHECK(b.is_active() == true);
	LT_CHECK(c.is_active() == false);
	LT_CHECK(d1.is_active() == true);
	LT_CHECK(d2.is_active() == false);
	LT_CHECK(d3.is_active() == false);

	sts = d2.activate();
	sts = d3.activate();

	sts = c.activate();
	LT_CHECK_EQ(sts, 0);
	LT_CHECK(a.is_active() == true);
	LT_CHECK(b.is_active() == true);
	LT_CHECK(c.is_active() == true);
	LT_CHECK(d1.is_active() == true);
	LT_CHECK(d2.is_active() == true);
	LT_CHECK(d3.is_active() == true);

	sts = d1.deactivate();
	LT_CHECK_EQ(sts, -2);

	sts = a.deactivate();
	LT_CHECK_EQ(sts, -2);

	LT_CHECK_EQ(a.is_controlled(), true);
	LT_CHECK_EQ(b.is_controlled(), true);
	LT_CHECK_EQ(c.is_controlled(), false);
	LT_CHECK_EQ(d1.is_controlled(), true);
	LT_CHECK_EQ(d2.is_controlled(), true);
	LT_CHECK_EQ(d3.is_controlled(), true);

	c.deactivate();
	LT_CHECK_EQ(a.is_controlled(), true);
	LT_CHECK_EQ(b.is_controlled(), false);
	LT_CHECK_EQ(c.is_controlled(), false);
	LT_CHECK_EQ(d1.is_controlled(), false);
	LT_CHECK_EQ(d2.is_controlled(), false);
	LT_CHECK_EQ(d3.is_controlled(), false);

	b.deactivate();
	LT_CHECK_EQ(a.is_controlled(), false);
	LT_CHECK_EQ(b.is_controlled(), false);
	LT_CHECK_EQ(c.is_controlled(), false);
	LT_CHECK_EQ(d1.is_controlled(), false);
	LT_CHECK_EQ(d2.is_controlled(), false);
	LT_CHECK_EQ(d3.is_controlled(), false);

	sts = a.deactivate();
	LT_CHECK_EQ(sts, 0);
	LT_CHECK_EQ(a.is_active(), false);
	sts = a.deactivate();	
	LT_CHECK_EQ(sts, 0);
	LT_CHECK_EQ(a.is_active(), false);

	sts = a.activate();
	LT_CHECK_EQ(sts, 0);
	LT_CHECK_EQ(a.is_active(), true);
	sts = a.activate();	
	LT_CHECK_EQ(sts, 0);
	LT_CHECK_EQ(a.is_active(), true);
}
LT_END_TEST(heimer_control_test)

LT_BEGIN_TEST(ralgo_test_suite, heimer_movement_test)
{
	int sts;

	heimer::stub_axis<float, float> ax("ax");
	heimer::axisctr axctr("axctr", &ax);

	sts = ax.activate();
	LT_CHECK_EQ(sts, 0);

	sts = axctr.activate();
	LT_CHECK_EQ(sts, 0);

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

	LT_CHECK(igris::early(maxspd, 4.0));
	LT_CHECK(igris::early(maxpos, 5.0));
}
LT_END_TEST(heimer_movement_test)

LT_BEGIN_TEST(ralgo_test_suite, heimer_absmovement_test)
{
	int sts;

	heimer::stub_axis<float, float> ax("ax");
	heimer::axisctr axctr("axctr", &ax);

	sts = ax.activate();
	LT_CHECK_EQ(sts, 0);

	sts = axctr.activate();
	LT_CHECK_EQ(sts, 0);

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

	LT_CHECK(igris::early(maxpos, 5.0));
}
LT_END_TEST(heimer_absmovement_test)


LT_BEGIN_TEST(ralgo_test_suite, heimer_movement_stop_test)
{
	int sts;

	heimer::stub_axis<float, float> ax("ax");
	heimer::axisctr axctr("axctr", &ax);

	sts = ax.activate();
	LT_CHECK_EQ(sts, 0);

	sts = axctr.activate();
	LT_CHECK_EQ(sts, 0);

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

	LT_CHECK_LT(ax.feedpos, 5);
}
LT_END_TEST(heimer_movement_stop_test)

LT_BEGIN_TEST(ralgo_test_suite, heimer_movement_hardstop_test)
{
	int sts;

	heimer::stub_axis<float, float> ax("ax");
	heimer::axisctr axctr("axctr", &ax);

	sts = ax.activate();
	LT_CHECK_EQ(sts, 0);

	sts = axctr.activate();
	LT_CHECK_EQ(sts, 0);

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

	LT_CHECK_LT(ax.feedpos, 5);
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
	LT_CHECK_EQ(sts, 0);
	sts = ax2.activate();
	LT_CHECK_EQ(sts, 0);
	sts = tand.activate();
	LT_CHECK_EQ(sts, 0);
	sts = axctr1.activate();
	LT_CHECK_EQ(sts, 0);
	sts = axctr2.activate();
	LT_CHECK_EQ(sts, 0);

	axctr1.set_speed(10);
	axctr2.set_speed(10);
	axctr1.set_accdcc(10,10);
	axctr2.set_accdcc(10,10);

	sts = axctr1.incmove(8);
	LT_CHECK_EQ(sts, 0);
	sts = axctr2.incmove(8);
	LT_CHECK_EQ(sts, 0);

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

	LT_CHECK(igris::early(8, ax1.feedpos, 1e-3));
	LT_CHECK(igris::early(4, ax2.feedpos, 1e-3));
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
	LT_CHECK_EQ(sts, 0);
	sts=ax2.activate();;
	LT_CHECK_EQ(sts, 0);
	sts=linint.activate();;
	LT_CHECK_EQ(sts, 0);

	linint.set_speed(10);
	linint.set_accdcc(100000, 100000);

	float target[2] = {10, 3};
	sts=linint.incmove(target);
	LT_CHECK_EQ(sts, 0);		

	LT_CHECK_EQ(linint.in_operate(), true);		

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

	LT_CHECK(ralgo::discrete_time() - t <= 1045);
	LT_CHECK(igris::early(10, ax1.feedpos, 1e-3));
	LT_CHECK(igris::early(3, ax2.feedpos, 1e-3));
}
LT_END_TEST(heimer_linint)


LT_BEGIN_TEST(ralgo_test_suite, heimer_control_panel) 
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

	LT_CHECK_EQ(ax1.is_active(), true);
	LT_CHECK_EQ(ax2.is_active(), true);
	LT_CHECK_EQ(axctr1.is_active(), true);
	LT_CHECK_EQ(axctr2.is_active(), true);

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

	dprln(ax1.ctrpos);
	LT_CHECK(igris::early(10, ax1.ctrpos));
}
LT_END_TEST(heimer_control_panel)
