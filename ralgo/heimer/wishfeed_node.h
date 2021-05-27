#ifndef RALGO_HEIMER_SIGNAL_WORKER_H
#define RALGO_HEIMER_SIGNAL_WORKER_H

#include <igris/container/array_view.h>
#include <ralgo/heimer/wishfeed.h>

#include <ralgo/linalg/matops.h>
#include <ralgo/linalg/matrix_view.h>

#define SIGWORKER_NAME_MAXSIZE 8
#define WISHFEED_NODE_MAXDIM 3

namespace heimer
{
	class wishfeed_node
	{
	protected:
		char _name[8];
	
		int _left_dim;
		int _right_dim;

		wishfeed * _left_signals[WISHFEED_NODE_MAXDIM];
		wishfeed * _right_signals[WISHFEED_NODE_MAXDIM];

	public:
		using getter_ptr = real*(wishfeed::*)();

		wishfeed_node() = default;

		void set_dim(int left, int right) 
		{
			_left_dim = left;
			_right_dim = right;
		}

		igris::array_view<wishfeed*> left_signals()
		{
			return { _left_signals, (size_t)_left_dim };
		}

		igris::array_view<wishfeed*> right_signals()
		{
			return { _right_signals, (size_t)_right_dim };
		}

		void bind_signals(
			igris::array_view<wishfeed*> left, 
			igris::array_view<wishfeed*> right
		) 
		{
			std::copy(left.begin(), left.end(), _left_signals);
			std::copy(right.begin(), right.end(), _right_signals);
		}

		virtual void serve_feed() = 0;
		virtual void serve_wish() = 0;

		ralgo::matrix_view<real> signals_as_matrix(
			igris::array_view<wishfeed *> inbuf, 
			getter_ptr getter, 
			real * signal_buffer)
		{
			int sigdim = inbuf[0]->size();
			int sigcount = inbuf.size();

			ralgo::matrix_view<real> signal {signal_buffer, sigcount, sigdim};

			for (int i = 0; i < sigcount; ++i)
			{
				wishfeed * sig = inbuf[i];
				real * data = (sig->*getter)();
				for (int j = 0; j < sigdim; ++j)
				{
					signal.at(i, j) = data[j];
				}
			}

			return signal;
		}

		void set_signals(
			const ralgo::matrix_view<real> & result, 
			getter_ptr getter, 
			igris::array_view<wishfeed *> outsigs)
		{
			int sigdim = outsigs[0]->size();

			for (int i = 0; i < outsigs.size(); ++i)
			{
				auto * sig = outsigs[i];
				auto * data = (sig->*getter)();
				for (int j = 0; j < sigdim; ++j)
				{
					data[j] = result.at(i, j);
				}
			}
		}
	};
}

#endif