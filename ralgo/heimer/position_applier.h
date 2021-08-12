/** 
	@file position_applier.h
*/

#ifndef RALGO_HEIMER_POSITION_APPLIER
#define RALGO_HEIMER_POSITION_APPLIER

namespace heimer 
{
	class position_applier 
	{
		robo::i_position_setter *   posset;	
		robo::i_position_feedback * posget;
		axis_state * state;

	public:
		position_applier(const char * name, robo::i_position_driver driver, axis_state * state) 
		{
			posset = driver;
			posget = driver;
			this->state = state;
		} 		

		
	};
}

#endif