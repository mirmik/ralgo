#ifndef VIRTDEVS_DEVICE_H
#define VIRTDEVS_DEVICE_H

#include <utility>
#include <igris/dprint.h>
#include <igris/sync/syslock.h>
#include <igris/container/array_view.h>

#include <igris/datastruct/dlist.h>

namespace virtdevs
{
	const std::pair<int, int> ErrorInDependDevice { -4000, 0 };
	const std::pair<int, int> DeviceIsNotActive { -4001, 0 };
	const std::pair<int, int> UndefinedStatus { -4002, 0 };
	const std::pair<int, int> NoErrorStatus { 0, 0 };

	using almmsg_t = std::pair<int, int>;

	extern dlist_head vitrdev_list;

	struct AlarmState
	{
		std::pair<int, int> almmsg = DeviceIsNotActive;
		bool in_alarm_state = true;

		std::pair<int, int> message()
		{
			return almmsg;
		}

		bool is_alarmed()
		{
			return in_alarm_state;
		}

		void allgood() 
		{
			almmsg = NoErrorStatus;
			in_alarm_state = false;
		}

		void set(bool alarmed, almmsg_t msg)
		{
			almmsg = msg;
			in_alarm_state = alarmed;
		}

		const char* status_string() 
		{
			return "undefined error";
		}
	};

	// Если понадобиться сделать другой тип ошибок,
	// сделать класс шаблонным и реализовать девайсы
	// как наследников общего предка.
	class device
	{
	public:
		dlist_head vitrdev_list_lnk = DLIST_HEAD_INIT(vitrdev_list_lnk);

	private:
		using alarm_t = std::pair<int, int>;
		//Status status = Status::Alarm;
		//Status self_status = Status::Ok;

		int ok_deps_counter = 0;
		AlarmState almstate;

		//alarm_t almmsg = DeviceIsNotActive;
		//alarm_t self_almmsg = DeviceIsNotActive;

		device * supervisor = nullptr;
		bool active = false;

		const char* m_name;

		//protected:
		//	void recursive_alarm(device * source, std::string text);
		//	void recursive_allgood();

	private:

		int take_control()
		{
			system_lock();
			for (device* dev : dependence())
			{
				if (dev->supervisor != nullptr || dev->active != true)
				{
					system_unlock();
					return -1;
				}
			}

			for (device* dev : dependence())
			{
				dev->supervisor = this;
				dev->supervisor_enabled_handle(this);
			}
			active = true;
			system_unlock();

			//reevaluate_alarm_status();
			return 0;
		}

		int put_control()
		{
			system_lock();
			active = false;
			for (device* dev : dependence())
			{
				dev->supervisor = nullptr;
				dev->supervisor_disabled_handle(this);
			}
			system_unlock();
		}

	public:
		virtual igris::array_view<device*> dependence() = 0;

		device(const char* name)
			: m_name(name)
		{
			dlist_add( &vitrdev_list_lnk, &vitrdev_list);
		}

		almmsg_t alarm_message() 
		{
			if (!is_active())
				return DeviceIsNotActive;

			if (dependence().size() != ok_deps_counter)
				return ErrorInDependDevice;

			return almstate.almmsg; 
		}

		bool is_alarmed()
		{
			return !is_active()
				   || almstate.is_alarmed()
			       || ok_deps_counter != dependence().size();
		}

		const char* name() { return m_name; }
		const char* name() const { return m_name; }

		void alarm(almmsg_t msg)
		{
			if (almstate.is_alarmed() && almstate.almmsg == msg)
				return;

			almstate.set(true, msg);

			alarm_handle();
			if (supervisor)
			{
				supervisor->ok_deps_counter--;
				supervisor->alarm_in_depend_device();
			}
		}

		void allgood()
		{
			if ( is_alarmed() )
			{
				almstate.allgood();

				if (is_ready()) 
					ready_handle();
				
				if (supervisor) 
				{
					supervisor->ok_deps_counter++;
					supervisor->allgood_in_depend_device();
				}
			}
		}

		int activate_device()
		{
			int sts;

			if ( is_active() )
				return 0;

			if (sts = take_control())
				return sts;

			if (sts = activate_device_handle())
			{
				put_control();
				return sts;
			}

			ok_deps_counter = 0;

			for(auto d : dependence()) 
			{
				if (d->is_ready()) 
				{
					ok_deps_counter++;
				}
			}

			return 0;
		}

		void deactivate_device()
		{
			if ( ! is_active() )
				return;

			if (supervisor && supervisor->is_active())
				supervisor->deactivate_device();

			put_control();

			deactivate_device_handle();
		}

		virtual void supervisor_enabled_handle(device* svisor) {}
		virtual void supervisor_disabled_handle(device* svisor) {}
		virtual int activate_device_handle() { return 0; }
		virtual void deactivate_device_handle() {}
		virtual void ready_handle() {}
		virtual void alarm_handle() {}

		int is_active() { return active; }
		int is_ready() { return is_active() && !is_alarmed(); }
		int is_controlled() { return supervisor != nullptr; }

		void alarm_in_depend_device() 
		{
			alarm_handle();
			//almstate.set(true, ErrorInDependDevice);	
		}

		void allgood_in_depend_device()
		{
			if (ok_deps_counter == dependence().size()
			        && !is_alarmed())
			{
				ready_handle();
			}
		}

		int get_ok_deps_counter() { return ok_deps_counter; }

		const char* status_string() 
		{
			return almstate.status_string();
		}

		/*void increment_supervisors_alarm_counter();
		void decrement_supervisors_alarm_counter();

		const alarm_t& alarm_message() { return almmsg; }

		virtual void alarm_handle()
		{
			dprln("virtdev::ALARM", name());
		}

		virtual void ready_handle()
		{
			dprln("virtdev::READY", name());
		}

		void reevaluate_alarm_status()
		{
			int alms = 0;

			system_lock();
			for (device * dev : dependence())
			{
				if (dev->status == Status::Alarm)
				{
					almmsg = ErrorInDependDevice;
					break;
				}
			}
			system_unlock();

			if (status == Status::Alarm)
			{
				alarm_handle();
			}
		}

		virtual void supervisor_enabled_handle() { dprln(name(), ": svisor enabled"); }
		virtual void supervisor_disabled_handle() { dprln(name(), ": svisor disabled"); }

		bool is_device_active() { return active; }
		};
		}
		*/
	};

	int cli_utility(int argc, char** argv, char* ret, int retmax);
}
#endif