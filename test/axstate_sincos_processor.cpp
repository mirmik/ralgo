#include <doctest/doctest.h>
#include <ralgo/heimer2/command.h>
#include <ralgo/heimer2/axstate_sincos_processor.h>
#include <ralgo/heimer2/distance.h>

#include <igris/math.h>

TEST_CASE("axstate_sincos_processor")
{

	SUBCASE("without_offset")
	{
		heimer_reinit();

		struct axis_state xl, yl, al, xr, yr, ar;
		struct axis_state * left[] = { &xl, &yl, &al };
		struct axis_state * right[] = { &xr, &yr, &ar };

		struct axstate_sincos_processor scproc;

		float angle = 60.f;

		xr.ctrpos = heimdist(10.f);
		yr.ctrpos = heimdist(20.f);
		ar.ctrpos = heimdeg(angle);

		xr.ctrvel = heimvel(1);
		yr.ctrvel = heimvel(2);
		ar.ctrvel = heimvel(1);

		axstate_sincos_processor_init(&scproc, "axproc", left, right, heimdist(10.f));
		signal_processor_serve(&scproc.proc, 0);

		CHECK_EQ(heimpos_cos(ar.ctrpos), doctest::Approx(cosf(deg2rad(angle))));
		CHECK_EQ(heimpos_sin(ar.ctrpos), doctest::Approx(sinf(deg2rad(angle))));
		CHECK_EQ(xl.ctrpos, doctest::Approx(heimdist(10.f + 10.f * cosf(deg2rad(angle)))));
		CHECK_EQ(yl.ctrpos, doctest::Approx(heimdist(20.f + 10.f * sinf(deg2rad(angle)))));
		CHECK_EQ(al.ctrpos, doctest::Approx(heimdeg(angle)));

		CHECK_EQ(xl.ctrvel, doctest::Approx(heimvel(1. - 10. * sinf(deg2rad(angle)))));
		CHECK_EQ(yl.ctrvel, doctest::Approx(heimvel(2. + 10. * cosf(deg2rad(angle)))));
		CHECK_EQ(al.ctrvel, doctest::Approx(heimvel(1)));

		xl.feedpos = xl.ctrpos;
		yl.feedpos = yl.ctrpos;
		al.feedpos = al.ctrpos;

		xl.feedvel = xl.ctrvel;
		yl.feedvel = yl.ctrvel;
		al.feedvel = al.ctrvel;

		signal_processor_feedback(&scproc.proc, 0);

		CHECK_EQ(xr.feedpos, heimdist(10.f));
		CHECK_EQ(yr.feedpos, heimdist(20.f));
		CHECK_EQ(ar.feedpos, heimdeg(angle));

		CHECK_EQ(xr.feedvel, heimvel(1));
		CHECK_EQ(yr.feedvel, heimvel(2));
		CHECK_EQ(ar.feedvel, heimvel(1));
	}


	SUBCASE("with_offset")
	{
		heimer_reinit();

		struct axis_state xl, yl, al, xr, yr, ar;
		struct axis_state * left[] = { &xl, &yl, &al };
		struct axis_state * right[] = { &xr, &yr, &ar };

		struct axstate_sincos_processor scproc;

		float angle = 60.f;

		xr.ctrpos = heimdist(10.f);
		yr.ctrpos = heimdist(20.f);
		ar.ctrpos = heimdeg(angle);

		xr.ctrvel = heimvel(1);
		yr.ctrvel = heimvel(2);
		ar.ctrvel = heimvel(1);

		axstate_sincos_processor_init(&scproc, "axproc", left, right, heimdist(10.f));
		axstate_sincos_processor_set_offset(
			&scproc,
			heimdist(1),
			heimdist(2),
			heimdist(0.1),
			heimdist(0.2)
		);
		signal_processor_serve(&scproc.proc, 0);

		CHECK_EQ(heimpos_cos(ar.ctrpos), doctest::Approx(cosf(deg2rad(angle))));
		CHECK_EQ(heimpos_sin(ar.ctrpos), doctest::Approx(sinf(deg2rad(angle))));
		CHECK_EQ(heimdist_restore(xl.ctrpos), doctest::Approx(10.f + 1 + 10.f * cosf(deg2rad(angle) + 0.2)));
		CHECK_EQ(heimdist_restore(yl.ctrpos), doctest::Approx(20.f + 2 + 10.f * sinf(deg2rad(angle) + 0.2)));
		CHECK_EQ(heimdist_restore(al.ctrpos), doctest::Approx(deg2rad(angle)+0.2+0.1));

		CHECK_EQ(heimvel_restore(xl.ctrvel), doctest::Approx(1. - (10.) * sinf(deg2rad(angle) + 0.2)));
		CHECK_EQ(heimvel_restore(yl.ctrvel), doctest::Approx(2. + (10.) * cosf(deg2rad(angle) + 0.2)));
		CHECK_EQ(heimvel_restore(al.ctrvel), doctest::Approx(1));

		xl.feedpos = xl.ctrpos;
		yl.feedpos = yl.ctrpos;
		al.feedpos = al.ctrpos;

		xl.feedvel = xl.ctrvel;
		yl.feedvel = yl.ctrvel;
		al.feedvel = al.ctrvel;

		signal_processor_feedback(&scproc.proc, 0);

		CHECK_EQ(heimdist_restore(xr.feedpos), doctest::Approx(10.f));
		CHECK_EQ(heimdist_restore(yr.feedpos), doctest::Approx(20.f));
		CHECK_EQ(heimdist_restore(ar.feedpos), doctest::Approx(deg2rad(angle)));

		CHECK_EQ(heimvel_restore(xr.feedvel), doctest::Approx(1));
		CHECK_EQ(heimvel_restore(yr.feedvel), doctest::Approx(2));
		CHECK_EQ(heimvel_restore(ar.feedvel), doctest::Approx(1));
	}

}
