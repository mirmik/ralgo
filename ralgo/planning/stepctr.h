#ifndef RALGO_MSERVO_CONTROLLER_H
#define RALGO_MSERVO_CONTROLLER_H

#include <limits>

namespace ralgo 
{
	class stepctr_server;

	class stepctr
	{ 
	public:
		int64_t impcurrent;
		int64_t imptarget;

		int32_t stepwidth;
		int32_t stepaccum;

		stepctr_server * parent;

	public:
		stepctr() 
			: 
			impcurrent(0),
			imptarget(0),
			stepwidth(std::numeric_limits<int32_t>::maximum()),
			stepaccum(0)
		{}

		// Установить ширину импульса в единицах инкремента.
		void set_stepwidth(int stepwidth) 
		{
			//if (parent->tdelta > stepwidth)
			//	BUG();

			this->stepwidth = stepwidth;
		}

		virtual void inc() = 0;
		virtual void dec() = 0;
	};

	class stepctr_server
	{
	public:
		int32_t tdelta; // < время между итерациями алгоритма (период прерывания)

		stepctr ** axes;
		int axtotal;

	public:
		stepctr_server(stepctr ** ptr, int total) 
		{
			axes = ptr;
			axtotal = total;
		}
	
		void serve() 
		{
			int diff;
			stepctr * ax;

			for (int i = 0; i < axtotal; ++i) 
			{
				ax = axes[i];

				if ((diff = ax->imptarget - ax->impcurrent) != 0) 
				{
					ax->stepaccum += tdelta;
					if (ax->stepaccum > ax->stepwidth) 
					{
						ax->stepaccum -= ax->stepwidth;
						if (diff > 0) 
						{
							ax->inc();
							++ax->impcurrent;
						}
						else 
						{
							ax->dec();
							--ax->impcurrent;	
						}

						if (ax->impcurrent == ax->imptarget)
							ax->stepaccum = 0;
					}
				}
			}
		}
	};
}

#endif