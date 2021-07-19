#include <ralgo/robo/quadgen4_arduino.h>

void quadgen4_arduino_init(struct quadgen4_arduino * qgen, uint8_t a, uint8_t b, uint8_t c, uint8_t d)
{
	qgen->apin0 = a;
	qgen->apin1 = b;
	qgen->bpin0 = c;
	qgen->bpin1 = d;
}

void quadgen4_arduino_set(struct quadgen4_arduino * qgen, uint8_t state)
{
	uint8_t setcode = graycode8(state);

	int a = (setcode & 0b01);
	int b = (setcode & 0b10);

	digitalWrite(qgen->apin0,  a);
	digitalWrite(qgen->apin1, !a);
	digitalWrite(qgen->bpin0,  b);
	digitalWrite(qgen->bpin1, !b);
}
