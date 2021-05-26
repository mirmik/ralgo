#ifndef RALGO_HEIMER_SIGNAL_WORKER_H
#define RALGO_HEIMER_SIGNAL_WORKER_H

#include <ralgo/heimer/wishfeed.h>

#define SIGWORKER_NAME_MAXSIZE 8
#define WISHFEED_NODE_MAXDIM 3

namespace heimer
{
	class wishfeed_node
	{
		char _name[8];
		WishfeedNodeType _type;

		int _left_dim;
		int _right_dim;

		wishfeed_union * _left_signals[WISHFEED_NODE_MAXDIM];
		wishfeed_union * _right_signals[WISHFEED_NODE_MAXDIM];

	public:
		wishfeed_node(int left_signals_total) {}

		igris::array_view<wishfeed_union*> & left_signals() override
		{
			return { _left_signals, left_dim };
		}

		igris::array_view<wishfeed_union*> & right_signals()  override
		{
			return { _right_signals, right_dim };
		}

		virtual int left_dim() = 0;
		virtual int right_dim() = 0;

		virtual wishfeed_union* left_signals() = 0;
		virtual wishfeed_union* right_signals() = 0;

		virtual void serve_feed() = 0;
		virtual void serve_wish() = 0;

	};
}

#endif