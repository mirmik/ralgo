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
			device * _controller = nullptr;
			igris::array_view<device*> _controlled;

			bool _is_busy = false;

		public:
			device(){}
			device(const char* name) : _name(name) {}			
			device(const char* name, igris::array_view<device*> controlled) : _controlled(controlled) {}
			device(const char* name, device ** table, size_t total) : _controlled(table, total) {}

			device * controller() { return _controller; }
			igris::array_view<device*> controlled() { return _controlled; }
			void set_name(const char * name) { _name = name; }
			bool is_busy() { return _is_busy; }
			const char* name() { return _name; }

			const char* controller_name() 
			{
				return _controller == nullptr? "null" : _controller->name();
			} 


			void set_controlled(igris::array_view<device*> controlled) { _controlled = controlled; }
			void set_controlled(device ** table, size_t total) { _controlled = { table, total }; }

			bool take_control(device * controller)
			{
				bool success = true;

				if (_controller == controller) 
					return true;
				
				if (is_busy())
					return false;

				for (auto dev : _controlled)
				{
					success = dev->take_control(controller);

					if ( !success )
						break;
				}

				if ( !success )
				{
					for (auto dev : _controlled)
					{
						dev->release_control(controller);
					}
				}
				else
				{
					_is_busy = true;
					this->_controller = controller;
				}

				return true;
			}

			bool take_control_force(device * controller)
			{
				if (_controller == controller) 
					return true;

				_controller = controller;
			
				for (auto dev : _controlled)
				{
					dev->take_control_force(controller);
				}
			}

			bool release_control(device * controller)
			{
				if (this->_controller == controller)
				{
					_controller =nullptr;
					for (auto dev : _controlled)
					{
						dev->release_control(controller);
					}
				}
			}

			bool release_control_force(device * controller)
			{
				_controller = nullptr;
				for (auto dev : _controlled)
				{
					dev->release_control(controller);
				}
			}

			bool take_control() { return take_control(this); }

			void take_control_force() { take_control_force(this); }
			void release_control_force() { release_control_force(this); }

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