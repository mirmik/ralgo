#ifndef RALGO_PLANNING_AXIS_INTERFACE_H
#define RALGO_PLANNING_AXIS_INTERFACE_H

/**
	Интерфейс управления базовыми функциями осей.
	Самые простые, часто применяемые функции, которые можно
	добавить в прототип интерфейса для обеспечения минимального
	набора функционала.
*/

namespace ralgo 
{
	template <class P, class Parameters>
	class axis_interface 
	{
	public:
		Parameters current_parameters;
		Parameters default_parameters;

		virtual int setup_params() { return -1; }
		int setup_default_params() 
		{
			current_parameters = default_parameters;
			return setup_params();
		} 

		void init_params(const Parameters& params) 
		{
			current_parameters = default_parameters = params;
		} 

		axis_interface(const Parameters& params) { init_params(params); }
		
		virtual int absolute_move(P pos, const Parameters& params) { return -1; }
		virtual int incremental_move(P pos, const Parameters& params) { return -1; }
		virtual int jog(int direction, const Parameters& params) { return -1; }
		
		virtual int stop(const Parameters& params) { return -1; }
		virtual int hardstop() { return stop(); }
		
		virtual int set_null_position() { return -1; }
		virtual int set_reference_position(P pos) { return -1; }
		
		int absolute_move(P pos) 
		{ 
			return absolute_move(pos, current_parameters); 
		}
		
		int incremental_move(P pos) 
		{ 
			return incremental_move(pos, current_parameters); 
		}
		
		int jog(int direction) 
		{ 
			return jog(direction, current_parameters); 
		}

		int stop() 
		{
			return stop(current_parameters);
		}
	};
}

#endif