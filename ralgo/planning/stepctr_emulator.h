#ifndef RALGO_STEPCTR_EMULATOR_H
#define RALGO_STEPCTR_EMULATOR_H

#include <ralgo/planning/stepctr.h>

namespace ralgo 
{
	class stepctr_emulator : public stepctr
	{
		int64_t pos = 0;

		void power(bool en) {}

		void inc() { ++pos; }
		void dec() { --pos; }
	};
}

#endif