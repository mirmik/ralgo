#ifndef RALGO_HEIMER_SIGNAL_WORKER_H
#define RALGO_HEIMER_SIGNAL_WORKER_H

#include <signal.h>

#define SIGWORKER_NAME_MAXSIZE 8


namespace heimer
{
	template <class Type>
	class wishfeed_node_basic
	{
		char _name[8];
		int type;

	public:
		wishfeed_node_basic(int left_signals_total) {}

		virtual int left_dim() = 0;
		virtual int right_dim() = 0;

		virtual wishfeed* left_signals() = 0;
		virtual wishfeed* right_signals() = 0;

		virtual void serve_feed() = 0;
		virtual void serve_wish() = 0;

	};

	template <class LeftType, class RightType, int LeftDim, int RightDim>
	class wishfeed_node : public
	{
		wishfeed<LeftType> * _left_signals[LeftDim];
		wishfeed<RightType> * _right_signals[RightDim];

	public:
		wishfeed_node() : wishfeed_node_basic(LeftDim, RightDim) {}

		igris::array_view<wishfeed*> & left_signals() override
		{
			return { _left_signals, LeftDim };
		}

		igris::array_view<wishfeed*> & right_signals()  override
		{
			return { _right_signals, RightDim };
		}

		int left_dim() override
		{
			return LeftDim;
		}

		int right_dim() override
		{
			return RightDim;
		}
	}
}

#endif