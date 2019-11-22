#ifndef RALGO_COORDKEEPER_H
#define RALGO_COORDKEEPER_H

namespace ralgo 
{
	template <class Position, int Dim>
	class coordkeeper 
	{
		int fixed[Dim];
		int planned[Dim];

	public:
		virtual int plan_new(igris::array_view<Position> pose);
	};
}

#endif