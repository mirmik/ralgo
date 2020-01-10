#ifndef RALGO_PLANNING_POSITION_READER_H
#define RALGO_PLANNING_POSITION_READER_H

namespace ralgo 
{
	class position_reader 
	{
	public:
		virtual int64_t read() = 0;
	};
}

#endif