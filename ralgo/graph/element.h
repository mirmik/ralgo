#ifndef RALGO_GRAPH_ELEMENT_H
#define RALGO_GRAPH_ELEMENT_H

namespace ralgo
{
	struct graph_element
	{
		int walk_counter;

		virtual int outputs_iterate() = 0;
		virtual int inputs_iterate() = 0;

		virtual int outputs_total() = 0;
		virtual int inputs_total() = 0;

	public:
		void reset()
		{
			walk_counter = 0;
		}
	}
}

#endif