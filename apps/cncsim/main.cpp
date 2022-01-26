#include <getopt.h>
#include <igris/time/systime.h>
#include <nos/input.h>
#include <nos/shell/argv.h>
#include <nos/io/string_writer.h>

#include <ralgo/cnc/interpreter.h>
#include <ralgo/cnc/planner.h>
#include <ralgo/cnc/revolver.h>
#include <ralgo/robo/stepper.h>

#include <crow/address.h>
#include <crow/gates/udpgate.h>
#include <crow/nodes/publisher_node.h>
#include <crow/nodes/service_node.h>
#include <crow/tower.h>

igris::ring<cnc::planner_block, 40> blocks;
igris::ring<cnc::control_shift, 400> shifts;

cnc::planner planner(&blocks, &shifts);
cnc::revolver revolver(&shifts);
cnc::interpreter interpreter(&blocks, &planner, &revolver);

#include <chrono>
#include <thread>

void planner_thread_function();
void revolver_thread_function();
void telemetry_thread_function();
std::thread revolver_thread;
std::thread planner_thread;
std::thread telemetry_thread;

auto crowker = crow::crowker_address();

using std::chrono::operator""ms;
using std::chrono::operator""us;

robo::stepper steppers[3];

crow::publisher publisher(crowker, "cncsim/mon/pose");
crow::service_node control_service(crowker, "cncsim/cli", +[]
    (char *cmd, int len, crow::service_node& srv)
{
	cmd[len] = 0;
	nos::println("input: ", std::string(cmd, len), "END");
    nos::string_buffer answer;
    interpreter.executor.execute(nos::tokens(cmd), answer);
    srv.reply(answer.str().data(), answer.str().size());
});

auto now() { return std::chrono::steady_clock::now(); }

void planner_thread_function()
{
    auto awake = now();
    while (1)
    {
        awake += 1ms;
        std::this_thread::sleep_until(awake);
        planner.serve();
    }
}

void telemetry_thread_function()
{
    auto awake = now();
    float poses[3];
    while (1)
    {
        awake += 10ms;
        std::this_thread::sleep_until(awake);
        for (int i = 0; i < 3; ++i)
            poses[i] = float(steppers[i].steps_count()) / 100;
        publisher.publish(igris::buffer(poses, sizeof(poses)));
    }
}

namespace heimer
{
    double fast_cycle_frequence() { return 10000; }
}

void revolver_thread_function()
{
    auto awake = now();
    while (1)
    {
        awake += 100us;
        std::this_thread::sleep_until(awake);
        revolver.serve();
    }
}

robo::stepper *steppers_ptrs[] = {&steppers[0], &steppers[1], &steppers[2]};
void configuration() { revolver.set_steppers(steppers_ptrs, 3); }

void print_help()
{
	printf(
	    "Usage: crowker [OPTION]...\n"
	    "\n"
	    "Common option list:\n"
		"  -h, --help            print this page\n"
		"  -p, --noprotect            disable global protection\n"
		"\n"
	);
}

int main(int argc, char ** argv)
{
	const struct option long_options[] =
	{
		{"help", no_argument, NULL, 'h'},
		{"noprotect", no_argument, NULL, 'p'},
		{NULL, 0, NULL, 0}
	};

	int long_index = 0;
	int opt = 0;

	while ((opt = getopt_long(argc, argv, "usvdibtn", long_options,
	                          &long_index)) != -1)
	{
		switch (opt)
		{
			case 'h':
				print_help();
				exit(0);

			case 'p':
				ralgo::global_protection = false;

			case 0:
				break;
		}
	}

	crow::create_udpgate();
	crow::start_spin();
    control_service.install_keepalive(2000);

	configuration();
	revolver_thread = std::thread(revolver_thread_function);
	planner_thread = std::thread(planner_thread_function);
	telemetry_thread = std::thread(telemetry_thread_function);

	std::this_thread::sleep_for(1000ms);

    interpreter.init_axes(3);
	interpreter.set_scale(ralgo::vector<double>{1,1,1});
	planner.set_gears({10000, 10000, 10000});
	//interpreter.set_saved_acc(10);
    //interpreter.set_saved_feed(10);
	interpreter.set_revolver_frequency(10000);

	/*interpreter.newline("cnc G01 X2 F5");
	interpreter.newline("cnc G01 Y2 F5");
	interpreter.newline("cnc G01 X5 F1");
	interpreter.newline("cnc G01 Y5 F5");
	interpreter.newline("cnc G01 Y5 Z3 F5");
	interpreter.newline("cnc G01 X20 F10");
	interpreter.newline("cnc G01 Y-20 F10");
	interpreter.newline("cnc G01 X-20 F10");*/

	while (1)
	{
		auto str = nos::readline();
		interpreter.newline(str);
	}
}
