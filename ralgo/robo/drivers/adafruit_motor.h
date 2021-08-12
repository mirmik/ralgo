#ifndef RALGO_ROBO_ADAFRUIT_MOTOR_DRIVER_H
#define RALGO_ROBO_ADAFRUIT_MOTOR_DRIVER_H

#include <stdint.h>
#include <ralgo/robo/motor.h>

#ifndef __ARDUINO__
// Mock
class Adafruit_DCMotor
{
public:
	void setSpeed(float) {}
	void run(int) {}
};
#define RELEASE 0
#define BACKWARD 0
#define FORWARD 1
#else
#include <Adafruit_DCMotor.h>
#endif

namespace robo
{
	class adafruit_motor_driver : public robo::motor
	{
		Adafruit_DCMotor* M;

		uint8_t setted_mode = RELEASE;
		bool reverse = false;

	public:
		void power(float pwr) override
		{
			if (reverse) pwr = -pwr;

			if (pwr == 0)
			{
				M->run(RELEASE);
				M->setSpeed(0);
				setted_mode = RELEASE;
				return;
			}

			if (pwr > 0)
			{
				if (setted_mode == BACKWARD) { M->run(RELEASE); }

				M->setSpeed(pwr * 255);

				if (setted_mode != FORWARD) { M->run(FORWARD); }

				setted_mode = FORWARD;
				return;
			}

			if (pwr < 0)
			{
				if (setted_mode == FORWARD) { M->run(RELEASE); }

				M->setSpeed((-pwr) * 255);

				if (setted_mode != BACKWARD) { M->run(BACKWARD); }

				setted_mode = BACKWARD;
			}
		}

		void stop() override
		{

			power(0);
		}
	};
}

#endif