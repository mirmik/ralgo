#include <ralgo/heimer/control.h>
#include <ralgo/heimer/stub_axis.h>
#include <ralgo/heimer/axisctr.h>
#include <ralgo/heimer/tandem.h>
#include <ralgo/heimer/linintctr.h>
#include <ralgo/heimer/polygon_checker.h>

#include <ralgo/heimer/command_center.h>

#include <igris/math.h>
#include <igris/datastruct/argvc.h>

#include <main.h>

LT_BEGIN_TEST(ralgo_test_suite, polygon_checker)
{
	bool sts;

	heimer::stub_axis<float, float> ax0("ax0");
	heimer::stub_axis<float, float> ax1("ax1");
	heimer::axis_node<float,float> * arr0[] = { &ax0, & ax1 };
	igris::array_view<heimer::axis_node<float,float> *> _arr0{arr0};
	heimer::linintctr<float,float,2> ig0("ig0", _arr0);

	ax0.set_current_phase(10, 0);
	ax1.set_current_phase(5, 0);

	sts=ax0.activate();
	sts=ax1.activate();
	sts=ig0.activate();
	CHECK_EQ(sts, 0);

	linalg::vec<float,2> vec[64];
	igris::array_view<linalg::vec<float,2>> parr(vec);
	heimer::plane_zone_checker<float> polygon(parr);
	ig0.coord_checker = &polygon;

	polygon.add({20,0});
	polygon.add({25,5});
	polygon.add({20,10});
	polygon.add({0,10});
	polygon.add({-5,5});
	polygon.add({0,0});

	ig0.set_speed(10);
	ig0.set_accdcc(10000, 10000);

	float target[2];
	target[0] = 10; target[1] = 3; CHECK_EQ(0, ig0.absmove(target));
	target[0] = 30; target[1] = 3; CHECK_EQ(-1, ig0.absmove(target));
	target[0] = -5; target[1] = 3; CHECK_EQ(-1, ig0.absmove(target));
	target[0] = -5; target[1] = 5; CHECK_EQ(0, ig0.absmove(target));
}

LT_BEGIN_TEST(ralgo_test_suite, polygon_difference_checker)
{
	bool sts;

	heimer::stub_axis<float, float> ax00("ax00");
	heimer::stub_axis<float, float> ax01("ax01");
	heimer::axis_node<float,float> * arr0[] = { &ax00, & ax01 };
	igris::array_view<heimer::axis_node<float,float> *> _arr0{arr0};
	heimer::linintctr<float,float,2> ig0("ig0", _arr0);

	heimer::stub_axis<float, float> ax10("ax10");
	heimer::stub_axis<float, float> ax11("ax11");
	heimer::axis_node<float,float> * arr1[] = { &ax10, & ax11 };
	igris::array_view<heimer::axis_node<float,float> *> _arr1{arr1};
	heimer::linintctr<float,float,2> ig1("ig1", _arr1);

	ax00.set_current_phase(10, 0);
	ax01.set_current_phase(5, 0);
	ax10.set_current_phase(20, 0);
	ax11.set_current_phase(10, 0);

	sts=ax00.activate();
	sts=ax01.activate();
	sts=ax10.activate();
	sts=ax11.activate();
	sts=ig0.activate();
	sts=ig1.activate();
	CHECK_EQ(sts, 0);

	linalg::vec<float,2> vec[64];
	igris::array_view<linalg::vec<float,2>> parr(vec);
	heimer::polygon_difference_checker<float,float> polygon(&ig0, &ig1, parr);
	ig0.coord_checker = &polygon;
	ig1.coord_checker = &polygon;

	polygon.add({10,-5});
	polygon.add({15,0});
	polygon.add({10,5});
	polygon.add({-10,5});
	polygon.add({-15,0});
	polygon.add({-10,-5});

	ig0.set_speed(10);
	ig0.set_accdcc(10000, 10000);
	ig1.set_speed(10);
	ig1.set_accdcc(10000, 10000);

	//ig0.feedback(); //nostate
	//ig1.feedback(); //nostate

	linalg::vec<float,2> v;
	ig0.current_point((float*)&v);
	CHECK_EQ(v, linalg::vec<float,2>{10,5});
	ig1.current_point((float*)&v);
	CHECK_EQ(v, linalg::vec<float,2>{20,10});

	float target[2];
	target[0] = 30; target[1] = 14; CHECK_EQ(0, ig0.absmove(target));
	target[0] = 30; target[1] = 16; CHECK_EQ(-1, ig0.absmove(target));
	target[0] = 6; target[1] = 10; CHECK_EQ(0, ig0.absmove(target));
	target[0] = 10; target[1] = 15; CHECK_EQ(0, ig0.absmove(target));
	target[0] = 30; target[1] = 5; CHECK_EQ(0, ig0.absmove(target));
	
	target[0] = -4; target[1] = 5; CHECK_EQ(0, ig1.absmove(target));
	target[0] = 25; target[1] = 5; CHECK_EQ(0, ig1.absmove(target));
	target[0] = 10; target[1] = 10; CHECK_EQ(0, ig1.absmove(target));
	target[0] = 26; target[1] = 5; CHECK_EQ(-1, ig1.absmove(target));
}