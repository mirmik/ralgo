#include <doctest/doctest.h>
#include <ralgo/heimer2/command.h>
#include <ralgo/heimer2/axis_state.h>

#include <string.h>
#include <string_view>

TEST_CASE("heimer_command")
{
	char buf[96];
	int sts;

	heimer_system_init();

	CHECK_EQ(heimer_signals_count(), 0);
	CHECK_EQ(heimer_signal_processors_count(), 0);

	heimer_command_exec_safe("signew axstate x", NULL, 0);

	CHECK_EQ(heimer_signals_count(), 1);
	CHECK_EQ(heimer_signal_processors_count(), 0);

	heimer_command_exec_safe("ctrnew axisctr xctr", NULL, 0);

	CHECK_EQ(heimer_signals_count(), 1);
	CHECK_EQ(heimer_signal_processors_count(), 1);

	sts = heimer_command_exec_safe("sig k info", buf, 96);
	CHECK_NE(sts, 0);

	sts = heimer_command_exec_safe("sig x Info", buf, 96);
	CHECK_NE(sts, 0);

	sts = heimer_command_exec_safe("sig x info", buf, 96);
	CHECK_EQ(sts, 0);
	CHECK_NE(strlen(buf), 0);
	CHECK_EQ(std::string_view(buf), "(ctrpos:0,ctrvel:0.000000,feedpos:0,feedspd:0.000000)");

	struct signal_head * sig = signal_get_by_name("x");
	axis_state_info(
		sig, 
		buf, 
		96);
	CHECK_EQ(std::string_view(buf), "(ctrpos:0,ctrvel:0.000000,feedpos:0,feedspd:0.000000)");
}