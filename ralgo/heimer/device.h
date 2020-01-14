#ifndef RALGO_HEIMER_DEVICE_H
#define RALGO_HEIMER_DEVICE_H

#include <igris/container/array_view.h>
#include <igris/dprint.h>

namespace ralgo
{
	namespace heimer
	{
		class device
		{
		public:
			device * _controller;
			igris::array_view<device*> _controlled;

			bool _is_busy = false;

		public:
			device * controller() { return _controller; }
			igris::array_view<device*> controlled() { return _controlled; }
			bool is_busy() { return _is_busy; }

			device(){}
			device(igris::array_view<device*> controlled) : _controlled(controlled) {}
			device(device ** table, size_t total) : _controlled(table, total) {}

			void set_controlled(igris::array_view<device*> controlled) { _controlled = controlled; }
			void set_controlled(device ** table, size_t total) { _controlled = { table, total }; }

			bool take_control(device * controller)
			{
				bool busy_signal = false;

				if (is_busy())
					return false;

				for (auto dev : _controlled)
				{
					busy_signal = take_control(this);

					if (busy_signal)
						break;
				}

				if (busy_signal)
				{
					for (auto dev : _controlled)
					{
						dev->release_control(this);
					}
				}
				else
				{
					_is_busy = true;
					this->_controller = controller;
				}

				return busy_signal;
			}

			bool release_control(device * controller)
			{
				if (this->_controller == controller)
				{
					for (auto dev : _controlled)
					{
						dev->release_control(this);
					}
				}
			}

			bool take_control() { return take_control(nullptr); }
		};
	}
}

#endif