#include <doctest/doctest.h>
#include <ralgo/heimer2/command.h>
#include <ralgo/heimer2/axstate_sincos_processor.h>
#include <ralgo/heimer2/distance.h>

TEST_CASE("axstate_sincos_processor") 
{
	heimer_reinit();

	struct axis_state xl, yl, al, xr, yr, ar;
	struct axis_state * left[] = { &xl, &yl, &al };
	struct axis_state * right[] = { &xr, &yr, &ar };

	struct axstate_sincos_processor scproc;

	xl->ctrpos = heimdist(0);
	yl->ctrpos = heimdist(0);
	al->ctrpos = heimdist(0);

	axstate_sincos_processor_init(&scproc, "axproc", left, right, heimdist(50), 0, 0);	
	signal_processor_serve(&scproc.proc, 0);

	CHECK_EQ(ar->ctrpos, heimdist(0));
	CHECK_EQ(xr->ctrpos, heimdist(0));
	CHECK_EQ(yr->ctrpos, heimdist(0));
}