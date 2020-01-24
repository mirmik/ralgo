#ifndef RALGO_HEIMER_DEVICE_H
#define RALGO_HEIMER_DEVICE_H

#include <igris/container/array_view.h>
#include <igris/dprint.h>

#include <nos/print.h>

namespace ralgo
{
	namespace heimer
	{
		class device
		{
		public:
			const char * _name = "unnamed";
			device * _controller;
			igris::array_view<device*> _controlled;

			bool _is_busy = false;

		public:
			device * controller() { return _controller; }
			igris::array_view<device*> controlled() { return _controlled; }
			void set_name(const char * name) { _name = name; }
			bool is_busy() { return _is_busy; }
			const char* name() { return _name; }

			device(){}
			device(igris::array_view<device*> controlled) : _controlled(controlled) {}
			device(device ** table, size_t total) : _controlled(table, total) {}

			void set_controlled(igris::array_view<device*> controlled) { _controlled = controlled; }
			void set_controlled(device ** table, size_t total) { _controlled = { table, total }; }

			bool take_control(device * controller)
			{
				bool success = true;

				if (_controller == this) 
					return true;
				
				if (is_busy())
					return false;

				for (auto dev : _controlled)
				{
					success = dev->take_control(this);

					if ( !success )
						break;
				}

				if ( !success )
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

				return true;
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

			bool take_control() { return take_control(this); }

			void print_controlled_devices(nos::ostream * os, int tabs = 0) 
			{
				os->fill('\t', tabs);
				os->println(name());
				for (auto d : _controlled) 
				{
					d->print_controlled_devices(os, tabs+1);
				}
			}
		};
	}
}

#endif