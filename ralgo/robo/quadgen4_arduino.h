#ifndef RALGO_QUADGENER_H
#define RALGO_QUADGENER_H

#include <stdint.h>
#include <igris/util/graycode.h>
#include <igris/compiler.h>

#if __has_include(<Arduino.h>)
#include <Arduino.h>
#else
// Затычка для того, чтобы скомпилировать функцию (для тестов)
// на машине отличной от Arduino
static void digitalWrite(uint8_t pin, uint8_t val) {}
#endif

struct quadgen4_arduino
{
	uint8_t apin0, apin1, bpin0, bpin1;
};

__BEGIN_DECLS

void quadgen4_arduino_init(struct quadgen4_arduino * qgen, uint8_t a, uint8_t b, uint8_t c, uint8_t d);

void quadgen4_arduino_set(struct quadgen4_arduino * qgen, uint8_t state);

__END_DECLS

#endif