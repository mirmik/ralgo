#include <doctest/doctest.h>
#include <ralgo/heimer2/command.h>

TEST_CASE("heimer_command") 
{
	heimer_system_init();

	CHECK_EQ(heimer_signals_count(), 0);
	CHECK_EQ(heimer_signal_processors_count(), 0);

	heimer_command_exec_safe("sig new axstate", NULL);

	CHECK_EQ(heimer_signals_count(), 1);
	CHECK_EQ(heimer_signal_processors_count(), 0);

	heimer_command_exec_safe("ctr new axisctr", NULL);

	CHECK_EQ(heimer_signals_count(), 1);
	CHECK_EQ(heimer_signal_processors_count(), 1);
}