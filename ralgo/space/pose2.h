#ifndef RALGO_GEOM_POSE2_H
#define RALGO_GEOM_POSE2_H

#include <ralgo/linalg/linalg.h>

namespace ralgo
{
	template <class T>
	class pose2
	{
		linalg::vec<T,2> lin;
		T                ang;
		
	public:
		pose2(linalg::vec<T,2> lin, T ang)
			: lin(lin), ang(ang) {}

		pose2 inverse()
		{
			return { linalg::rot<T>(-ang, -lin), -ang };
		}

		pose2 operator * (const pose2& oth)
		{
			return pose2(
			           linalg::rot<T>(ang, oth.lin) + lin,
			           ang + oth.ang);
		}

		/*pose2 operator() (const pose2& oth) 
		{
			return (*this)(oth);
		}*/

		linalg::vec<T,2> transform_vector(const linalg::vec<T,2>& vec) const
		{
			return linalg::rot(ang, vec);
		}

		linalg::vec<T,2> transform_point(const linalg::vec<T,2>& vec) const
		{
			return linalg::rot(ang, vec) + lin;
		}
	};

	template <class T>
	T vector_angle(linalg::vec<T,2> vec) 
	{
		return atan2(vec.y, vec.x);
	}
}

#endif