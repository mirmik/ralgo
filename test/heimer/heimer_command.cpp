#include <doctest/doctest.h>
#include <ralgo/heimer/command.h>
#include <ralgo/heimer/axis_state.h>

#include <string.h>
#include <string_view>

using namespace heimer;

TEST_CASE("command")
{
	char buf[96];
	int sts;

	heimer_system_init();

	CHECK_EQ(heimer_signals_count(), 0);
	CHECK_EQ(heimer_signal_processors_count(), 0);

	heimer::command_exec_safe("signew axstate x", NULL, 0);

	CHECK_EQ(heimer_signals_count(), 1);
	CHECK_EQ(heimer_signal_processors_count(), 0);

	heimer::command_exec_safe("ctrnew axisctr xctr", NULL, 0);

	CHECK_EQ(heimer_signals_count(), 1);
	CHECK_EQ(heimer_signal_processors_count(), 1);

	sts = heimer::command_exec_safe("sig k info", buf, 96);
	CHECK_NE(sts, 0);

	sts = heimer::command_exec_safe("sig x Info", buf, 96);
	CHECK_NE(sts, 0);

	sts = heimer::command_exec_safe("sig x info", buf, 96);
	CHECK_EQ(sts, 0);
	CHECK_NE(strlen(buf), 0);

	struct signal_head * sig = signal_get_by_name("x");
	sig->info(
		buf, 
		96);
	CHECK_EQ(sts, 0);
	CHECK_NE(strlen(buf), 0);
}