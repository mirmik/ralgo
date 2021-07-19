#ifndef RALGO_HEIMER2_AXISCTR_H
#define RALGO_HEIMER2_AXISCTR_H

#include <ralgo/heimer2/axis_state.h>
#include <ralgo/heimer2/distance.h>
#include <ralgo/trajectory/linetraj.h>
#include <ralgo/disctime.h>

enum axis_controller_status 
{
	AXIS_CONTROLLER_STOP,
	AXIS_CONTROLLER_MOVE
};

struct axis_controller 
{
	float vel; // скорость в единицах  
	float acc; 
	float dcc;

	double gain;

	int64_t backlim; // Расстояние в единицах с фиксированной точкой.
	int64_t forwlim; // Расстояние в единицах с фиксированной точкой.

	enum axis_controller_status operation_status;

	void (* operation_start_handler)(void * priv, struct axis_controller * ax);
	void (* operation_finish_handler)(void * priv, struct axis_controller * ax);
	void * operation_handlers_priv;

	int spattern_enabled;

	struct line_trajectory lintraj;
	struct trajectory * curtraj;

	struct axis_state * controlled;
};

__BEGIN_DECLS


void axis_controller_set_gain(struct axis_controller * axctr, double gain);

void axis_controller_set_limits_external(struct axis_controller * axctr, double back, double forw);
void axis_controller_set_limits_internal_fixed(struct axis_controller * axctr, int64_t back, int64_t forw);

void axis_controller_set_controled(struct axis_controller * axctr, struct axis_state * state);
void axis_controller_set_velocity_external(struct axis_controller * axctr, float vel);
void axis_controller_set_velocity_internal(struct axis_controller * axctr, float vel);
void axis_controller_set_accdcc_external(struct axis_controller * axctr, float acc, float dcc);
void axis_controller_set_accdcc_internal(struct axis_controller * axctr, float acc, float dcc);

void axis_controller_incmove(struct axis_controller * axctr, double dist_real);
void axis_controller_absmove(struct axis_controller * axctr, double pos_real);

void axis_controller_serve(struct axis_controller * axctr, disctime_t time);

__END_DECLS

#endif