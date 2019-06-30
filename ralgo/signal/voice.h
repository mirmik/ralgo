#ifndef RALGO_SIGNAL_VOICE_H
#define RALGO_SIGNAL_VOICE_H

namespace ralgo 
{
	static inline constexpr double hz2mel(double hz) { return 1127 * log(1 + (hz / 700)); }
	static inline constexpr double mel2hz(double mel) { return 700 * (exp(mel/1127) - 1); }
}

#endif