#include <ralgo/heimer/speed_phaser.h>
#include <ralgo/heimer/stepctr.h>
#include <ralgo/heimer/speed_phaser_axis.h>
#include <ralgo/heimer/stepctr.h>
#include <ralgo/heimer/linear_interpolator.h>

#include <ralgo/heimer/xyalpha_controller.h>

#include <nos/fprint.h>

#include <thread>
#include <chrono>
#include <mutex>

using namespace std::literals::chrono_literals;

int main(int argc, char* argv[]) 
{
	ralgo::heimer::stepctr_emulator<float, int64_t, float> emul0;	
	ralgo::heimer::stepctr_emulator<float, int64_t, float> emul1;	
	emul0.set_name("emul0");
	emul1.set_name("emul1");

	ralgo::heimer::speed_phaser_axis<float, int64_t, float> ax0(&emul0);
	ralgo::heimer::speed_phaser_axis<float, int64_t, float> ax1(&emul1);
	ax0.set_name("ax0");
	ax1.set_name("ax1");

	ralgo::heimer::axis_device<float,float> * linint_axes[] = { &ax0, &ax1 };	
	ralgo::heimer::linear_interpolator<2, float, float> linint("linint", linint_axes);
	linint.set_name("linint");

	ralgo::heimer::xyalpha_controller<float,float> k2;

//	linint.take_control();
//	linint.print_controlled_devices(nos::current_ostream);

	emul0.set_deltatime(/*ticks_per_second*/10000);
	emul0.set_gain(4196000);
	emul0.set_gear(10000);

	emul1.set_deltatime(/*ticks_per_second*/10000);
	emul1.set_gain(4196000);
	emul1.set_gear(10000);

	ax0.set_speed(1);
	ax0.set_position_compensate(0.001);
	ax0.set_accdcc_value(1, 1);
	//ax0.incmove(10);

	ax1.set_speed(1);
	ax1.set_position_compensate(0.01);
	//ax1.incmove(10);


	if (argc == 1) {
		linint.set_speed(1);
		linint.set_accdcc_value(1, 1);
		linint.incmove({ -10, 10 });
	}
	else 
	{
		ax0.set_speed(3);
		ax0.set_accdcc_value(1, 1);
		ax0.incmove(-10);
	//	ax0.stop();
	}

	auto start = std::chrono::system_clock::now();
	auto waituntil = start;

	std::thread drvthr { [&]()
	{
		while(true) 
		{
			emul0.serve();
			emul1.serve();
			waituntil = waituntil + std::chrono::microseconds(100);
			std::this_thread::sleep_until(waituntil);
		}
	}};

	std::mutex mtx;

	std::thread axthr { [&]()
	{
		while(true) 
		{
			mtx.lock();
			ax0.serve();
			mtx.unlock();
			//ax1.serve();
			//linint.serve();
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
	}};

	std::thread logthr { [&]()
	{
		while(true) 
		{
			mtx.lock();
			nos::fprintln("emul0: {} {} {} ax0: {}", emul0.setted_speed(), emul0.feedback_position(), emul0.feedback_position_internal(), ax0.phase());
			nos::fprintln("emul1: {} {} {} ax1: {}", emul1.setted_speed(), emul1.feedback_position(), emul1.feedback_position_internal(), ax1.phase());
			
			mtx.unlock();
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
	}};

	std::thread wthr { [&]()
	{
		std::this_thread::sleep_for(4000ms);
		mtx.lock();
		ax0.stop();	
			mtx.unlock();	

		//while(1);
	}};

	drvthr.join();
}