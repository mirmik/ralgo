#ifndef RALGO_LIMIT_SWITCH_H
#define RALGO_LIMIT_SWITCH_H

namespace ralgo 
{
	class limit_switch 
	{
	public:
		igris::delegate<void> limdlg;

		void emit() 
		{
			limdlg();
		}
	};
}

#endif