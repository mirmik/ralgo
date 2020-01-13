#include <ralgo/heimer/speed_phaser.h>

#include <nos/fprint.h>

#include <thread>
#include <chrono>


int main() 
{
	ralgo::heimer::speed_phaser_emulator<int64_t, float> emul0;	

	emul0.set_deltatime(/*ticks_per_second*/10000);
	emul0.set_gain(4196000);
	emul0.set_speed(1);

	std::thread drvthr { [&]()
	{
		while(true) 
		{
			emul0.serve();
			std::this_thread::sleep_for(std::chrono::microseconds(100));
		}
	}};

	std::thread logthr { [&]()
	{
		while(true) 
		{
			nos::fprintln("emul0: {} {}", emul0.speed(), emul0.feedback_position());
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
	}};

	drvthr.join();
}