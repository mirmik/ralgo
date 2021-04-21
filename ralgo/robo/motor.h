#ifndef RALGO_ROBO_MOTOR_H
#define RALGO_ROBO_MOTOR_H

namespace ralgo {
	namespace robo {
		struct motor {
			virtual void power(float pwr) = 0;
			virtual void stop() { power(0); }
		};
	}
}

#endif