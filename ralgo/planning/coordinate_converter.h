#ifndef RALGO_COORDINATE_CONVERTER_H
#define RALGO_COORDINATE_CONVERTER_H

namespace ralgo 
{
	template <class Internal, class External>
	class basic_coordinate_converter 
	{
		// Convert external coordinate to internal system
		virtual Internal ext2int(External external) = 0;

		// Convert internal coordinate to external system
		virtual External int2ext(Internal internal) = 0;
	};

	template <class Internal, class External>
	class coordinate_converter 
	{
		// Пределы содержаться во внешнем типе, потому что так
		// их гораздо проще задавать и считывать. Кроме того,
		// проверке подлежат именно задания, которые приходят снаружи.

		External maxlimit;
		External minlimit;

	public:
		void set_limits(External maxlimit, External minlimit) 
		{
			assert(maxlimit >= minlimit);

			this->maxlimit = maxlimit;
			this->minlimit = minlimit;
		}

		// Convert external coordinate to internal system
		Internal ext2int(External arg) override 
		{

		}

		// Convert internal coordinate to external system
		External int2ext(Internal arg) override 
		{


			return 
		}
	}
}

#endif