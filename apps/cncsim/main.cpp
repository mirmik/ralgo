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
std::thread revolver_thread;
std::thread planner_thread;

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

void revolver_thread_function()
{
	auto start = now();
	auto awake = now();

	while (1)
	{
		awake += 100us; 
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

	interpreter.newline("G01 X10 Y20 F5");

	while (1)
	{
		auto str = nos::readline();
		interpreter.newline(str);
	}
}