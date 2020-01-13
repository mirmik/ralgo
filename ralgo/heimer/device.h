#ifndef RALGO_HEIMER_DEVICE_H
#define RALGO_HEIMER_DEVICE_H

namespace ralgo
{
	class device
	{
	public:
		device * _controller;
		igris::array_view<device> _controlled;

		bool _is_busy = false;

	public:
		device * controller() { return _controller; }
		igris::array_view<device> controlled() { return _controlled; }
		bool is_busy() { return _is_busy; }		

		device(igris::array_view<device> controlled) : controlled(controlled) {}
		device(device * table, size_t total) : controlled(table, total) {}

		bool take_control(device * controller)
		{
			bool busy_signal;

			if (is_busy) return false;

			for (auto dev : controlled)
			{
				busy_signal = take_control(this);
				if (busy_signal) break;
			}

			if (busy_signal)
			{
				for (auto dev : controlled)
				{
					dev->release_control(this);
				}
			}
			else 
			{
				is_busy = true;
				this->controller = controller;
			}
		}

		bool release_control(device * controller) 
		{
			if (this->controller == controller) 
			{
				for (auto dev : controlled) 
				{
					dev->release_control(this);
				}
			}
		}

		bool take_control() { return take_control(nullptr); }
	};
}

#endif