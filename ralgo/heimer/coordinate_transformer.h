/** @file */

#ifndef HELIX_COORDINATE_TRANSFORMER_H
#define HELIX_COORDINATE_TRANSFORMER_H

#include <ralgo/helix/signal.h>

namespace helix
{
	class phase_transformer 
	{
		std::vector<widewish_node_basic<Signal> *> workers;

	public:

		void create_wishfeed(const char * name) 
		{
			signals.emplace_back(name, );
		}
	};
}

#endif