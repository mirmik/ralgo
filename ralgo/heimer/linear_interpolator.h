#ifndef RALGO_HEIMER_LINEAR_INTERPOLATOR_H
#define RALGO_HEIMER_LINEAR_INTERPOLATOR_H

namespace ralgo
{
	namespace heimer
	{
		template<class Position, class Speed>
		class linear_interpolator : public heimer::device
		{
		public:
			linear_interpolator(
			    igris::array_view<heimer::device*> axes)
			{
				set_controlled(axes);				
			}

			axis_device<float,float> * get_axis(int index) 
			{
				return static_cast<axis_device<float,float>*>(controlled()[index]);
			}

			void incmove(
				igris::array_view<Position> dist
			) 
			{
				take_control();
			}
		};
	}
}

#endif