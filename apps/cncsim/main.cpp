#include <nos/input.h>
#include <igris/systime.h>

#include <ralgo/cnc/revolver.h>
#include <ralgo/cnc/planner.h>
#include <ralgo/cnc/interpreter.h>

#include <ralgo/robo/stepper.h>

igris::ring<cnc::planner_block, 40> blocks;
igris::ring<cnc::control_shift, 400> shifts;

cnc::interpreter interpreter(&blocks);
cnc::planner planner(&blocks, &shifts);
cnc::revolver revolver(&shifts);

#include <thread>
#include <chrono>

void planner_thread_function();
void revolver_thread_function();
void telemetry_thread_function();
std::thread revolver_thread;
std::thread planner_thread;
std::thread telemetry_thread;

using std::chrono::operator""ms;
using std::chrono::operator""us;

robo::stepper steppers[3]; 

auto now()
{
	return std::chrono::steady_clock::now();
}

void planner_thread_function()
{
	auto start = now();
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
	auto start = now();
	auto awake = now();

	while (1)
	{
		awake += 100ms; 
		std::this_thread::sleep_until(awake);

		nos::println(steppers[0].steps_count(), steppers[1].steps_count(), steppers[2].steps_count());
	}
}

void revolver_thread_function()
{
	auto start = now();
	auto awake = now();

	while (1)
	{
		awake += 100us; 
		//nos::println(
		//	std::chrono::duration_cast<std::chrono::milliseconds>(now().time_since_epoch()).count() - 
		//	std::chrono::duration_cast<std::chrono::milliseconds>(awake.time_since_epoch()).count());
		std::this_thread::sleep_until(awake);
	
		revolver.serve();
	}
}

robo::stepper * steppers_ptrs[] = { &steppers[0], &steppers[1], &steppers[2] };
void configuration() 
{
	revolver.set_steppers(steppers_ptrs, 3);
}

int main(int argc, char ** argv)
{
	configuration();
	revolver_thread = std::thread(revolver_thread_function);
	planner_thread = std::thread(planner_thread_function);
	telemetry_thread = std::thread(telemetry_thread_function);

	std::this_thread::sleep_for(1000ms);

	interpreter.gains[0] = 4194304./10000.;
	interpreter.gains[1] = 4194304./10000.;
	interpreter.gains[2] = 4194304./10000.;

	interpreter.task_acc = 10;
	interpreter.revolver_frequency = 10000;
	//interpreter.newline("G01 X2 F5");
	//interpreter.newline("G01 Y2 F5");
	//interpreter.newline("G01 X5 F1");
	//interpreter.newline("G01 Y5 F5");
	interpreter.newline("G01 Y5 Z3 F5");

	planner.total_axes = 3;

	while (1)
	{
		auto str = nos::readline();
		interpreter.newline(str);
	}
}