#ifndef RALGO_CNC_INTERPRETER_H
#define RALGO_CNC_INTERPRETER_H

#include <igris/container/ring.h>
#include <ralgo/cnc/planblock.h>

namespace cnc 
{
	class interpreter 
	{
	public:
		igris::ring<planner_block> * blocks;

		interpreter(igris::ring<planner_block> * blocks) : blocks(blocks) {}

		void newline(const std::string & line) 
		{

		}
	};
}

#endif