#include <ralgo/heimer/control.h>
#include <ralgo/heimer/stub_axis.h>
#include <ralgo/heimer/axisctr.h>

#include <igris/math.h>

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
		ax.update_state();
		
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
		ax.update_state();
		
		axctr.serve();
		ax.serve();

		auto pos = ax.feedpos; 
		auto spd = ax.feedspd;

		//dprln(pos, spd);

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
		ax.update_state();
		
		axctr.serve();
		ax.serve();

		auto pos = ax.feedpos; 
		auto spd = ax.feedspd;

		//dprln(pos, spd);

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
