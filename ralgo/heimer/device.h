#ifndef RALGO_HEIMER_DEVICE_H
#define RALGO_HEIMER_DEVICE_H

#include <igris/container/array_view.h>
#include <igris/dprint.h>

#include <nos/print.h>
#include <igris/datastruct/dlist.h>

#define CONTROL_SUCCESS true
#define CONTROL_ERROR false

namespace ralgo
{
	namespace heimer
	{
		class device;

		class controlled 
		{		
		public:		
			device * _controller = nullptr;
			device * controller() { return _controller; }
			
			virtual bool take_control(device * controller) = 0;
			virtual bool take_control_force(device * controller) = 0;

			virtual void release_control(device * controller) = 0;
			virtual void release_control_force(device * controller) = 0;

			virtual const char* name() = 0;
			virtual bool is_device_controller() { return false; } 
		};

		class device : public controlled
		{
		public:
			static dlist_head devices_list;
			dlist_head lnk = DLIST_HEAD_INIT(lnk);

			const char * _name = "unnamed";
			//igris::array_view<controlled*> _controlled;

			bool _is_busy = false;

		public:
			device(const char* name) : _name(name) { dlist_add(&lnk, &devices_list); }

			//igris::array_view<heimer::controlled*> controlled() { return _controlled; }
			void set_name(const char * name) { _name = name; }
			bool is_busy() { return _is_busy; }
			const char* name() { return _name; }

			const char* controller_name() 
			{
				return _controller == nullptr? "null" : _controller->name();
			} 


			//void set_controlled(igris::array_view<heimer::controlled*> c) { _controlled = c; }
			//void set_controlled(heimer::controlled ** table, size_t total) { _controlled = { table, total }; }

			bool take_control(device * controller) override
			{
				dprln("a");
				bool success = true;

				if (_controller == controller) 
					return true;
				
				if (is_busy())
					return false;

				for (auto dev : controlled_devices())
				{
					dprln(controlled_devices().size());
					dprln("b");
					success = dev->take_control(controller);

					if ( !success )
						break;
				}

				if ( !success )
				{
					dprln("c");
					for (auto dev : controlled_devices())
					{
						dev->release_control(controller);
					}
				}
				else
				{
					dprln("d");
					_is_busy = true;
					this->_controller = controller;
					after_take_control_handle();
				}
				dprln("e");

				return success;
			}

			bool take_control_force(device * controller) override
			{
				if (_controller == controller) 
					return true;

				_controller = controller;
			
				for (auto dev : controlled_devices())
				{
					dev->take_control_force(controller);
				}

				return true;
			}

			void release_control(device * controller) override
			{
				if (this->_controller == controller)
				{
					_controller =nullptr;
					for (auto dev : controlled_devices())
					{
						dev->release_control(controller);
					}
				}
			}

			void release_control_force(device * controller) override
			{
				_controller = nullptr;
				for (auto dev : controlled_devices())
				{
					dev->release_control(controller);
				}
			}

			bool take_control() { return take_control(this); }
			void take_control_force() { take_control_force(this); }

			void release_control() { release_control_force(this); }
			void release_control_force() { release_control_force(this); }

			void print_controlled_devices(nos::ostream * os, int tabs = 0) 
			{
				os->fill('\t', tabs);
				os->println(name());
				for (auto d : controlled_devices()) 
				{
					if (d->is_device_controller())
						((device*)d)->print_controlled_devices(os, tabs+1);
					else
						os->println(d->name());		
				}
			}

			bool is_device_controller() override { return true; } 

			static void debug_print_list() 
			{
				device * dev;
				dlist_for_each_entry_reverse(dev, &devices_list, lnk) 
				{
					dprln(dev->name());
				}
			}

		protected:
			virtual void after_take_control_handle() = 0;
			virtual igris::array_view<controlled*> controlled_devices() = 0;
		};
	}
}

#endif