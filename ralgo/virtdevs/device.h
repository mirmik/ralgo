#ifndef VIRTDEVS_DEVICE_H
#define VIRTDEVS_DEVICE_H

#include <igris/dprint.h>
#include <igris/sync/syslock.h>
#include <igris/container/array_view.h>

namespace virtdevs 
{
	const std::pair<int,int> ErrorInDependDevice { -4000, 0 };
	const std::pair<int,int> DeviceIsNotActive { -4001, 0 };

	enum class Status 
	{
		Ok,
		Alarm
	};

	// Если понадобиться сделать другой тип ошибок, 
	// сделать класс шаблонным и реализовать девайсы
	// как наследников общего предка.
	class device 
	{
		using alarm_t = std::pair<int,int>;
		Status status = Status::Alarm;		

		int alarmed_deps = 0;
		alarm_t almmsg = DeviceIsNotActive;

		device * supervisor = nullptr;
		virtual igris::array_view<device*> dependence() = 0;

		bool active = false;

		const char* m_name;
		
	protected:
		void recursive_alarm(device * source, std::string text);
		void recursive_allgood();

	public:
		device(const char* name) 
			: m_name(name) 
		{}

		const char* name() { return m_name; }
		const char* name() const { return m_name; }

		void alarm(alarm_t almmsg);
		void allgood();

		void increment_supervisors_alarm_counter();
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
				dev->supervisor_enabled_handle();
			}			
			active = true;
			system_unlock(); 

			reevaluate_alarm_status();
			return 0;
		}

		int disable_control() 
		{
			system_lock();
			active = false;
			for (device* dev : dependence()) 
			{
				dev->supervisor = nullptr;
				dev->supervisor_disabled_handle();
			}			
			system_unlock();
		} 

		virtual void supervisor_enabled_handle() { dprln(name(), ": svisor enabled"); }
		virtual void supervisor_disabled_handle() { dprln(name(), ": svisor disabled"); }

		bool is_device_active() { return active; }
	};
}

#endif