#ifndef RALGO_STEPCTR_EMULATOR_H
#define RALGO_STEPCTR_EMULATOR_H

#include <ralgo/planning/speed_phaser.h>

namespace ralgo 
{
	class stepctr_emulator : 
		public stepctr_speed_phaser<int64_t>
	{
		//int64_t pos = 0;
	public:

		stepctr_emulator(const char* str) {}

		void power(bool en) {}

		void inc() {}
		void dec() {}
	};
}

#endif