#include <doctest/doctest.h>
#include <ralgo/heimer/axstate_pid_processor.h>


TEST_CASE("axstate_pid_processor") 
{
	heimer::axis_state x("x");
	heimer::scalar_signal t("t");

	heimer::axstate_pid_processor proc("pidproc");
	proc.set_left(&x);
	proc.set_right(&t);

	t.value = 10;

	proc.activate(0);
	proc.serve(0);
	CHECK_EQ(x.ctrpos, 0);
	proc.serve(1);
	CHECK_NE(x.ctrpos, 0);
	CHECK_NE(x.ctrvel, 0);
	proc.serve(2);
	proc.serve(3);
	proc.serve(4);
}