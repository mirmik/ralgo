#ifndef RALGO_HEIMER2_AXISCTR_H
#define RALGO_HEIMER2_AXISCTR_H

#include <ralgo/heimer/axisctr_approval.h>
#include <ralgo/heimer/axis_state.h>
#include <ralgo/trajectory/linetraj.h>
#include <ralgo/disctime.h>

#include <ralgo/heimer/signal_processor.h>

struct axis_settings
{
	struct axis_state * controlled;
	int limits_enabled;
	position_t backlim;
	position_t forwlim;
	sf_position_t sfpos;
	double gain;
};

struct axis_controller : public signal_processor
{
	velocity_t     vel; // скорость в единицах
	acceleration_t acc;
	acceleration_t dcc;

	int operation_finished_flag;
	int release_control_flag;

	void (* operation_start_handler)(void * priv, struct axis_controller * ax);
	void (* operation_finish_handler)(void * priv, struct axis_controller * ax);
	void * operation_handlers_priv;

	int spattern_enabled;

	struct line_trajectory lintraj;
	struct trajectory * curtraj;

	struct axisctr_approval ** approvals;
	int approvals_total;

	int dim;
	struct axis_settings * settings;

public:
	void feedback(disctime_t time) override;
	void serve(disctime_t time) override;
	int command(int argc, char ** argv, char * output, int outmax) override;
	void deinit() override;
	struct signal_head * iterate_left(struct signal_head *) override;

	void init(
	    const char * name,
	    struct axis_settings * setings,
	    int dim
	);
	void deinit(struct signal_processor * sigproc);

	void set_handlers(
	    void * operation_handlers_priv,
	    void (* operation_start_handler)(void * priv, struct axis_controller * ax),
	    void (* operation_finish_handler)(void * priv, struct axis_controller * ax)
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

__BEGIN_DECLS


struct axis_controller * create_axis_controller(const char * name, int dim);

__END_DECLS

#endif