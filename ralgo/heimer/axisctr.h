#ifndef RALGO_HEIMER2_AXISCTR_H
#define RALGO_HEIMER2_AXISCTR_H

#include <ralgo/heimer/axisctr_approval.h>
#include <ralgo/heimer/axis_state.h>
#include <ralgo/trajectory/linetraj.h>
#include <ralgo/disctime.h>

#include <ralgo/heimer/signal_processor.h>

namespace heimer
{
	struct axis_settings
	{
	public:
		struct axis_state * controlled;
		int limits_enabled;
		position_t backlim;
		position_t forwlim;
		sf_position_t sfpos;
		double gain;
	};

	class axis_controller : public signal_processor
	{
	public:
		velocity_t     vel; // скорость в единицах
		acceleration_t acc;
		acceleration_t dcc;

		union
		{
			uint8_t flags;
			struct
			{
				uint8_t operation_finished_flag : 1;
				uint8_t release_control_flag : 1;
				uint8_t dynamic_resources : 1;
				uint8_t spattern_enabled : 1;
			} f;
		};


		void (* operation_start_handler)(void * priv, axis_controller * ax);
		void (* operation_finish_handler)(void * priv, axis_controller * ax);
		void * operation_handlers_priv;

		struct line_trajectory lintraj;
		struct trajectory * curtraj;

		struct axisctr_approval ** approvals;
		int approvals_total;

		int dim;
		struct axis_settings * settings;

	public:
		axis_controller() = default;
		axis_controller(
		    const char * name,
		    struct axis_settings * setings,
		    int dim
		);

		int feedback(disctime_t time) override;
		int serve(disctime_t time) override;
		int command(int argc, char ** argv, char * output, int outmax) override;
		void deinit() override;
		signal_head * iterate_left(signal_head *) override;
		signal_head * iterate_right(signal_head *) override;

		void init(
		    const char * name,
		    struct axis_settings * setings,
		    int dim
		);
		void deinit(struct signal_processor * sigproc);

		void set_handlers(
		    void * operation_handlers_priv,
		    void (* operation_start_handler)(void * priv, axis_controller * ax),
		    void (* operation_finish_handler)(void * priv, axis_controller * ax)
		);

		void set_gain(double * gain);

		void set_limits_external(double * back, double * forw);
		void set_velocity_external(float vel);
		void set_accdcc_external(float acc, float dcc);

		void set_limits_internal(position_t * back, position_t * forw);
		void set_velocity_internal(velocity_t vel);
		void set_accdcc_internal(acceleration_t acc, acceleration_t dcc);

		void set_controlled(struct axis_state ** state);
		void release_controlled();

		int incmove(disctime_t current_time, double * dist_real);
		int absmove(disctime_t current_time, double * pos_real);

		float feedpos_external(int axno);
		float ctrpos_external(int axno);
		float ctrvel_external(int axno);

		int command(
		    struct signal_processor * sigproc,
		    int argc, char ** argv,
		    char * output,
		    int outmax);

	private:
		int _absmove(
		    disctime_t curtim,
		    position_t * curpos,
		    position_t * tgtpos,
		    double extdist);

		void finish_trajectory(disctime_t time, position_t * ctrpos);

	};

	axis_controller * create_axis_controller(const char * name, int dim);
}

#endif