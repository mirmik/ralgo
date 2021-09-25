/**
	@file velocity_applier.h
*/

#ifndef RALGO_HEIMER_VELOCITY_APPLIER
#define RALGO_HEIMER_VELOCITY_APPLIER

#include <ralgo/robo/stepper_controller.h>
#include <ralgo/heimer/axis_state.h>
#include <ralgo/heimer/signal_processor.h>

namespace heimer
{
	/**
		Применятет данные объектов axis_state к объектам реализующим интерфейсы датчиков
		скоростей и положений библиотеки robo. Комплементарно корректирует установленную
		скорость по данных о положении.
	*/
	class velocity_applier : public signal_processor
	{
		robo::i_velocity_setter * controlled_velset;
		robo::i_velocity_feedback * controlled_velget;
		robo::i_position_feedback * controlled_posget;
		axis_state * state;

		position_t deviation_error_limit = 10000;
		float compkoeff = 0; /// Коэффициент комплементарного фильтра.

		// Количество импульсов в системной единице (миллиметре или радиане).
		float gear = 1;
		
		disctime_t last_time;

		robo::fixed_frequency_stepper_controller * stepctr;

	public:
		// debug
		velocity_t compspd;

	public:
		velocity_applier();

		velocity_applier(
		    const char * name,
		    robo::fixed_frequency_stepper_controller * stepctr
		);

		velocity_applier(
		    const char * name,
		    robo::fixed_frequency_stepper_controller * stepctr,
		    axis_state * state
		);

		void set_gear(float gear);

		void init(
		    const char * name,
		    robo::fixed_frequency_stepper_controller * stepctr,
		    axis_state * state
		);

		int bind(int argc, char ** argv, char * output, int outmax);

		//void set_gain(float gain) { this->gain = gain; }
		void set_compkoeff(float ck) { this->compkoeff = ck; }

		void set_compkoeff_timeconst(float T) { this->compkoeff = 1. / discrete_time_frequency() / T; }

		int feedback(disctime_t time) override;
		int serve(disctime_t time) override;
		int info(char * ans, int anslen) override;
		int command(int argc, char ** argv, char * output, int outmax) override;
		void deinit() override;
		signal_head * iterate_left(signal_head *) override;
		signal_head * iterate_right(signal_head *) override;
	};
}

#endif