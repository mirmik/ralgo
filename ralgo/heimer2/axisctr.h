#ifndef RALGO_HEIMER2_AXISCTR_H
#define RALGO_HEIMER2_AXISCTR_H

#include <ralgo/heimer2/axis_state.h>
#include <ralgo/heimer2/distance.h>
#include <ralgo/trajectory/linetraj.h>
#include <ralgo/disctime.h>

#include <ralgo/heimer2/signal_processor.h>

struct axis_controller 
{
	struct signal_processor sigproc;

	float vel; // скорость в единицах  
	float acc; 
	float dcc;

	double gain;

	int64_t backlim; // Расстояние в единицах с фиксированной точкой.
	int64_t forwlim; // Расстояние в единицах с фиксированной точкой.

	int operation_finished_flag;

	void (* operation_start_handler)(void * priv, struct axis_controller * ax);
	void (* operation_finish_handler)(void * priv, struct axis_controller * ax);
	void * operation_handlers_priv;

	int spattern_enabled;

	struct line_trajectory lintraj;
	struct trajectory * curtraj;

	struct axis_state * controlled;

	// private:
	int64_t _line_trajectory_spos;
	int64_t _line_trajectory_fpos;
};

__BEGIN_DECLS

void axis_controller_init(struct axis_controller * axctr, const char * name);
void axis_controller_set_handlers(
	struct axis_controller * axctr,
	void * operation_handlers_priv,
	void (* operation_start_handler)(void * priv, struct axis_controller * ax),
	void (* operation_finish_handler)(void * priv, struct axis_controller * ax)
);

void axis_controller_set_gain(struct axis_controller * axctr, double gain);

void axis_controller_set_limits_external(struct axis_controller * axctr, double back, double forw);
void axis_controller_set_velocity_external(struct axis_controller * axctr, float vel);
void axis_controller_set_accdcc_external(struct axis_controller * axctr, float acc, float dcc);

void axis_controller_set_limits_internal(struct axis_controller * axctr, int64_t back, int64_t forw);
void axis_controller_set_velocity_internal(struct axis_controller * axctr, float vel);
void axis_controller_set_accdcc_internal(struct axis_controller * axctr, float acc, float dcc);

void axis_controller_set_controlled(struct axis_controller * axctr, struct axis_state * state);

int axis_controller_incmove(struct axis_controller * axctr, disctime_t current_time, double dist_real);
int axis_controller_absmove(struct axis_controller * axctr, disctime_t current_time, double pos_real);

void axis_controller_serve(struct signal_processor * sigproc, disctime_t time);

float axis_controller_ctrpos_external(struct axis_controller * axctr);
float axis_controller_ctrvel_external(struct axis_controller * axctr);

struct axis_controller * create_axis_controller(struct axis_controller * axctr, const char * name);

__END_DECLS

#endif