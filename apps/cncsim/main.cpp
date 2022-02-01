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
#include <mutex>

igris::ring<cnc::planner_block> blocks{40};
igris::ring<cnc::control_shift> shifts{400};

cnc::planner planner(&blocks, &shifts);
cnc::revolver revolver(&shifts);
cnc::interpreter interpreter(&blocks, &planner, &revolver);

std::mutex mtx;
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
	std::lock_guard<std::mutex> lock(mtx);
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
        mtx.lock();
        planner.serve();
        mtx.unlock();
    }
}

void telemetry_thread_function()
{
    int64_t poses[NMAX_AXES];
    auto awake = now();
    while (1)
    {
        awake += 10ms;
        std::this_thread::sleep_until(awake);
        for (size_t i = 0; i < interpreter.get_axes_count(); ++i)
            poses[i] = int64_t(steppers[i].steps_count());
        publisher.publish(igris::buffer(poses, sizeof(int64_t) * interpreter.get_axes_count()));
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
        
        mtx.lock();
        revolver.serve();
        mtx.unlock();
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
		"Crow services:\n"
		"	cncsim/cli - cncsim control\n"
		"Crow pulbics:\n"
		"	cncsim/poses - cncsim axes poses\n"
		"\n"
	);
}

int main(int argc, char ** argv)
{
	publisher.set_qos(0,0);
	control_service.set_qos(2, 50);
	control_service.set_rqos(2, 50);

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

	interpreter.init_axes(3);
	interpreter.set_scale(ralgo::vector<double>{1,1,1});
	planner.set_gears({10000, 10000, 10000});
	interpreter.set_revolver_frequency(10000);

	crow::create_udpgate();
	crow::start_spin();
    
    control_service.install_keepalive(2000, true);

	configuration();
	revolver_thread = std::thread(revolver_thread_function);
	planner_thread = std::thread(planner_thread_function);
	telemetry_thread = std::thread(telemetry_thread_function);

	std::this_thread::sleep_for(100ms);

	while (1)
	{
		auto str = nos::readline();
		interpreter.newline(str);
	}
}
