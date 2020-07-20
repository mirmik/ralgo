#ifndef RALGO_HEIMER_CONTROL_H
#define RALGO_HEIMER_CONTROL_H

#include <igris/datastruct/dlist.h>
#include <igris/util/bug.h>
#include <igris/shell/conscmd.h>

namespace ralgo
{
	namespace heimer
	{
		extern dlist_head control_info_node_list;
	
		class external_control_slot;

		// Тот, кто выполняет работу.
		class control_served
		{
			bool _is_activate = false;

		public:
			virtual void serve_impl() = 0;
			void serve() { if (_is_activate) serve_impl(); }

			int activate()
			{
				if (_is_activate)
					return 0;

				if (try_activate_impl()) 
				{
					return -1;
				}

				else
				{
					_is_activate = true;
					on_activate_handle();
					return 0;
				}
			}

			int deactivate()
			{
				if (!_is_activate)
					return 0;


				if (try_deactivate_impl()) 
				{
					return -1;
				}
				
				else
				{
					_is_activate = false;
					on_deactivate_handle();
					return 0;
				}
			}

			virtual int try_activate_impl() = 0;
			virtual int try_deactivate_impl() = 0;

			bool is_active() { return _is_activate; }

			virtual void on_activate_handle() = 0;
			virtual void on_deactivate_handle() = 0;
		};


		// Тот, кто контролирует.
		class external_controller
		{
		public:
			virtual void control_interrupt_from(external_control_slot * slot) = 0;
			virtual external_control_slot* iterate(external_control_slot*) = 0;

			int take_control();
			int release_control();
		};


		// Тот, кого контролируют.
		class external_control_slot
		{
		public:
			external_controller* extctr = nullptr;

			int try_take_external_control(external_controller * controller)
			{
				if (try_take_external_control_impl(controller))
				{
					return -1;
				}
				else
				{
					extctr = controller;
					return 0;
				}
			}

			int try_release_external_control(external_controller * controller)
			{
				if (try_release_external_control_impl(controller))
				{
					return -1;
				}
				else
				{
					extctr = nullptr;
					return 0;
				}
			}

			bool is_extern_controlled() { return (bool) extctr; }
			external_controller* extcontroller() { return extctr; }

			virtual int try_take_external_control_impl(external_controller * controller) = 0;
			virtual int try_release_external_control_impl(external_controller * controller) = 0;

			void external_control_interrupt() { extctr->control_interrupt_from(this); }
		};


		class control_info_node
		{
		public:
			const char * _mnemo;
			dlist_head lnk;

			control_served * srv;
			external_controller * ctr;
			external_control_slot * slt;

			control_info_node(const char* mnemo, control_served* srv, external_controller * ctr, external_control_slot * slt)
				: _mnemo(mnemo),
				  srv(srv),
				  ctr(ctr),
				  slt(slt)
			{
				dlist_add_prev(&lnk, &control_info_node_list);
			}

			const char* mnemo() { return _mnemo; }
			virtual void print_info() { BUG(); }
		};

		extern igris::console_command info_node_commands[];
	}
}

#endif