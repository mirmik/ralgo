#include <ralgo/trajectory/traj1d.h>
#include <ralgo/trajectory/speed_deformer.h>

#include <igris/math.h>
#include <main.h>

LT_BEGIN_TEST(ralgo_test_suite, traj_speed_deformer)
{
	ralgo::speed_deformer deform;
	deform.set_time_pattern(0.2, 0.2);
	CHECK(igris::early(deform.real_spd, 1.25));
	CHECK(igris::early(deform.f_time, 1));
	CHECK_EQ(deform.posmod(1), 1);
	CHECK_EQ(deform.spdmod(0.5), deform.real_spd);

	deform.set_speed_pattern(0.2,0.2);
	CHECK(igris::early(deform.real_spd, 1));
	CHECK(igris::early(deform.f_time, 1.2));
	CHECK_EQ(deform.posmod(deform.f_time), 1);
	CHECK_LT(deform.posmod(1), 1);
	CHECK_EQ(deform.spdmod(0.5), 1);
}
LT_END_TEST(traj_speed_deformer)

LT_BEGIN_TEST(ralgo_test_suite, traj_traj1_nominal_speed)
{
	ralgo::traj1d_line<float, float> traj;

	ralgo::traj1d_nominal_speed_params<float,float> nm_params = 
	{
		.stim = 1000,
		.spos = 10.0,
		.fpos = 20.0,
		.speed = 1.0,
		.acc = 1.0,
		.dcc = 1.0,
	};

	traj.init_nominal_speed_mode(&nm_params);

	int sts;
	float pos, spd;

	sts = traj.attime(1000, pos, spd);
	CHECK_EQ(sts, 0);
	CHECK_EQ(pos, 10);
	CHECK_EQ(spd, 0);

	sts = traj.attime(1500, pos, spd);
	CHECK_EQ(sts, 0);
	CHECK(igris::early(spd, 0.5));

	sts = traj.attime(2000, pos, spd);
	CHECK_EQ(sts, 0);
	CHECK(igris::early(spd, 1));
	
	sts = traj.attime(4000, pos, spd);
	CHECK_EQ(sts, 0);
	CHECK(igris::early(spd, 1));
	
	sts = traj.attime(11999, pos, spd);
	CHECK_EQ(sts, 0);
	CHECK(igris::early(spd, 0, 1e-2));
	CHECK(igris::early(pos, 20, 1e-2));

	sts = traj.attime(12000, pos, spd);
	CHECK_EQ(sts, 1);
	CHECK(igris::early(spd, 0));
	CHECK(igris::early(pos, 20));

	sts = traj.attime(20000, pos, spd);
	CHECK_EQ(sts, 1);
	CHECK(igris::early(spd, 0));
	CHECK(igris::early(pos, 20));
}
LT_END_TEST(traj_traj1_nominal_speed)

LT_BEGIN_TEST(ralgo_test_suite, traj_traj1_timestamp)
{
	ralgo::traj1d_line<float, float> traj;

	ralgo::traj1d_timestamp_params<float,float> ts_params = 
	{
		.stim = 1000,
		.ftim = 10000,
		.acctime = 1000,
		.dcctime = 1000,
		.spos = 10.0,
		.fpos = 20.0
	};

	traj.init_timestamp_mode(&ts_params);

	int sts;
	float pos, spd;

	sts = traj.attime(1000, pos, spd);
	CHECK_EQ(sts, 0);
	CHECK(igris::early(spd, 0));
	CHECK(igris::early(pos, 10));	

	sts = traj.attime(10000, pos, spd);
	CHECK_EQ(sts, 1);
	CHECK(igris::early(spd, 0));
	CHECK(igris::early(pos, 20));	

	sts = traj.attime(9999, pos, spd);
	CHECK_EQ(sts, 0);
	CHECK(igris::early(spd, 0, 1e-2));
	CHECK(igris::early(pos, 20, 1e-2));	
}
LT_END_TEST(traj_traj1_timestamp)


LT_BEGIN_TEST(ralgo_test_suite, traj_traj1_nominal_speed_mmm)
{
	ralgo::traj1d_line<float, float> traj;

	ralgo::traj1d_nominal_speed_params<float,float> nm_params = 
	{
		.stim = 1000,
		.spos = 10.0,
		.fpos = 11.0,
		.speed = 2.0,
		.acc = 0.5,
		.dcc = 0.5,
	};

	traj.init_nominal_speed_mode(&nm_params);

	int sts;
	float pos, spd;

	sts = traj.attime(1000, pos, spd);
	CHECK_EQ(sts, 0);
	CHECK_EQ(pos, 10);
	CHECK_EQ(spd, 0);

	sts = traj.attime(2000, pos, spd);
}
LT_END_TEST(traj_traj1_nominal_speed_mmm)
