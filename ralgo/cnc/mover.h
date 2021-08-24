#ifndef RALGO_CNC_MOVER_H
#define RALGO_CNC_MOVER_H

namespace cnc 
{
	class mover 
	{
	public:
		robo::stepper steppers[CNC_MAXIMUM_AXES];

	public:
		void interrupt_handle();
	}
}

#endif