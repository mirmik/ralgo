#ifndef RALGO_HEIMER_LINEAR_INTERPOLATOR_H
#define RALGO_HEIMER_LINEAR_INTERPOLATOR_H

namespace ralgo
{
	namespace heimer
	{
		template<class Position, class Speed>
		class linear_interpolator : public heimer::device
		{
		protected:
			union {
				igris::array_view<axis_device<Position, Speed>*> _axes;
				igris::array_view<axis_device<Position, Speed>*> _controled;
			};

		public:
			linear_interpolator(
			    igris::array_view<axis_device<Position, Speed>*> axes)
				: _axes(axes)
			{}

			void incmove() 
			{}
		};
	}
}

#endif