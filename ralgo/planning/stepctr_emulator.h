#ifndef RALGO_STEPCTR_EMULATOR_H
#define RALGO_STEPCTR_EMULATOR_H

#include <ralgo/planning/stepctr.h>

namespace ralgo 
{
	class stepctr_emulator : public stepctr
	{
		//int64_t pos = 0;
	public:

		stepctr_emulator(const char* str) : stepctr(str) {}

		void power(bool en) {}

		void inc() {}
		void dec() {}
	};
}

#endif