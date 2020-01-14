#include <ralgo/heimer/speed_phaser.h>
#include <ralgo/heimer/stepctr.h>
#include <ralgo/heimer/speed_phaser_axis.h>
#include <ralgo/heimer/stepctr.h>
#include <ralgo/heimer/linear_interpolator.h>

#include <nos/fprint.h>

#include <thread>
#include <chrono>


int main() 
{
	ralgo::heimer::stepctr_emulator<float, int64_t, float> emul0;	

	emul0.set_deltatime(/*ticks_per_second*/10000);
	emul0.set_gain(4196000);
	emul0.set_gear(10000);
	//emul0.set_speed(1);

	ralgo::heimer::speed_phaser_axis<float, int64_t, float> ax0(&emul0);
	ax0.set_speed(1);
	ax0.set_position_compensate(0.01);
	ax0.incmove(10);

	auto start = std::chrono::system_clock::now();
	auto waituntil = start;

	std::thread drvthr { [&]()
	{
		while(true) 
		{
			emul0.serve();
			waituntil = waituntil + std::chrono::microseconds(100);
			std::this_thread::sleep_until(waituntil);
		}
	}};

	std::thread axthr { [&]()
	{
		while(true) 
		{
			ax0.serve();
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
	}};

	std::thread logthr { [&]()
	{
		while(true) 
		{
			nos::fprintln("emul0: {} {} {} ax0: {}", emul0.setted_speed(), emul0.feedback_position(), emul0.feedback_position_internal(), ax0.phase());
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
	}};

	drvthr.join();
}