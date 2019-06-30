#ifndef RALGO_CONVOLUTION_H
#define RALGO_CONVOLUTION_H

#include <ralgo/interpolate.h>
#include <igris/util/bug.h>

namespace ralgo 
{
	namespace signal 
	{
		class window 
		{
		protected:
			double strt;
			double fini;

		public:
			window(double strt, double fini) : strt(strt), fini(fini) {}

			//virtual double convolution(double * vals, double * stamp, int n);
			//virtual double convolution(float * vals, float * stamp, int n);

			//void map(double * vals, double * stamp, int n);
		};

		class triangle_window : public window
		{
		public:
			triangle_window(double strt, double fini) : window(strt, fini) {}

			std::vector<double> keypoints() { return { strt, (strt + fini) / 2, fini }; }

			template <class V>
			std::vector<double> keypoints_map(const V& vec) 
			{
				std::vector<double> r;
				std::vector<double> kpoints = keypoints();
				ralgo::vecops::stamp_union_window(vec, kpoints, std::back_inserter(r));
				return r;
			}
		};
	}
}

#endif