#include <doctest/doctest.h>
#include <ralgo/heimer2/command.h>
#include <ralgo/heimer2/axstate_sincos_processor.h>
#include <ralgo/heimer2/distance.h>

#include <igris/math.h>

TEST_CASE("axstate_sincos_processor") 
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

	xr.ctrvel = 1;
	yr.ctrvel = 2;
	ar.ctrvel = 1;

	axstate_sincos_processor_init(&scproc, "axproc", left, right, heimdist(10.f));	
	signal_processor_serve(&scproc.proc, 0);

	CHECK_EQ(heimdist(10), 167772160);
	CHECK_EQ(heimpos_cos(ar.ctrpos), doctest::Approx(cosf(deg2rad(angle))));
	CHECK_EQ(heimpos_sin(ar.ctrpos), doctest::Approx(sinf(deg2rad(angle))));
	CHECK_EQ(xl.ctrpos, heimdist(10.f + 10.f * cosf(deg2rad(angle))));
	CHECK_EQ(yl.ctrpos, heimdist(20.f + 10.f * sinf(deg2rad(angle))));
	CHECK_EQ(al.ctrpos, heimdeg(angle));

	CHECK_EQ(xl.ctrvel, doctest::Approx(1. - 10. * sinf(deg2rad(angle))));
	CHECK_EQ(yl.ctrvel, doctest::Approx(2. + 10. * cosf(deg2rad(angle))));
	CHECK_EQ(al.ctrvel, 1);
}
