#ifndef RALGO_IMPULSE_POSITION_DRIVER_H
#define RALGO_IMPULSE_POSITION_DRIVER_H

#include <ralgo/lintrans.h>
#include <ralgo/servo/impulse_writer.h>
#include <ralgo/servo/position_driver.h>

namespace ralgo
{
	//Класс управления шаговым двигателем или сервоусилителем.
	template <typename P = float, typename V = float, typename A = float>
	struct impulse_position_driver : public ralgo::position_driver<P, V, A>
	{
		// ralgo::position_reader<P> * posreader;
		ralgo::impulse_writer<P, V> *impwriter;

		double scale_factor;

		void serve(const ralgo::phase<P, V, A> &phs) override
		{
			// P curpos = posreader->current_position();
			P tgtpos = phs.pos;
			V tgtspd = phs.spd;

			// assert(tgtspd > 0);

			impwriter->write(tgtpos * scale_factor, tgtspd * scale_factor);
		}
	};

} // namespace ralgo

#endif