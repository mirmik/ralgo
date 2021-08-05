#ifndef RALGO_HEIMER_PHASE_MUX_PROCESSOR_H
#define RALGO_HEIMER_PHASE_MUX_PROCESSOR_H

#include <ralgo/heimer/phase_signals.h>
#include <ralgo/heimer/signal_processor.h>

namespace heimer 
{
	template <int Dim>
	class phase_mux_processor : public signal_processor
	{
		axis_state * leftside[Dim];
		phase_signal<Dim> * rightside;

	public:
		phase_mux_processor(const char * name) : signal_processor(name, Dim, 1) {}

        int feedback(disctime_t) 
        {
        	for (int i = 0; i < Dim; ++i) 
        	{
        		rightside->feedpos[i] = leftside[i]->feedpos;
        		rightside->feedvel[i] = leftside[i]->feedvel;
        	}
        	return 0;
        }

        int serve(disctime_t) 
        {
        	for (int i = 0; i < Dim; ++i) 
        	{
        		leftside[i]->ctrpos = rightside->ctrpos[i];
        		leftside[i]->ctrvel = rightside->ctrvel[i];
        	}
        	return 0;
        }

		signal_head * leftsig(int i) { return leftside[i]; }
		signal_head * rightsig(int) { return rightside; }
		void set_leftsig(int i, signal_head * it) { leftside[i] = static_cast<axis_state*>(it); }
		void set_rightsig(int, signal_head * it) { rightside = static_cast<phase_signal<Dim>*>(it); }
		int leftsigtype(int) { return SIGNAL_TYPE_AXIS_STATE; }
		int rightsigtype(int) { return SIGNAL_TYPE_PHASE_SIGNAL_BASE + Dim - 1; }
	};
}

#endif