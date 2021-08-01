/** @brief */

#ifndef RALGO_HEIMER_AXSTATE_SIGNAL_PROCESSOR_H
#define RALGO_HEIMER_AXSTATE_SIGNAL_PROCESSOR_H

#include <ralgo/heimer/signal_processor.h>
#include <ralgo/heimer/axis_state.h>

namespace heimer
{
	class axstate_signal_processor : public signal_processor
	{
	private:
		struct axis_state ** _leftside = nullptr;
		struct axis_state ** _rightside = nullptr;
		int _rightdim;
		int _leftdim;

	public:
		axstate_signal_processor() = default;
		axstate_signal_processor(const char *);

		signal_head * iterate_left(signal_head *) override;
		signal_head * iterate_right(signal_head *) override;

		void attach_leftside_table(axis_state ** table, int dim);
		void attach_rightside_table(axis_state ** table, int dim);

		axis_state * leftax(int i) { return _leftside[i]; }
		axis_state * rightax(int i) { return _rightside[i]; }

		int leftdim() { return _leftdim; }
		int rightdim() { return _rightdim; }

		void set_leftside(axis_state ** arr);
		void set_rightside(axis_state ** arr);
	};

	int axstate_signal_processor_bindleft(axstate_signal_processor * axctr, int argc, char ** argv, char * output, int outmax);
	int axstate_signal_processor_bindright(axstate_signal_processor * axctr, int argc, char ** argv, char * output, int outmax);
}

#endif