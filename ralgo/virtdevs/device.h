#ifndef VIRTDEVS_DEVICE_H
#define VIRTDEVS_DEVICE_H

#include <vector>
#include <string>

#include <igris/dprint.h>
//#include <igris/event/multiple_delegate.h>

namespace virtdevs 
{
	enum class Status 
	{
		Ok,
		Alarm
	};

	class device 
	{
		Status status = Status::Alarm;		
		std::string m_status_string;

		std::vector<device *> supervisors;
		std::vector<device *> depends;
		
		int alarmed_deps = 0;

//		igris::multiple_delegate<> alarm_handle; 
//		igris::multiple_delegate<> ready_handle;

		std::string m_name;
		
	protected:
		void recursive_alarm(device * source, std::string text);
		void recursive_allgood();

	public:
		device(const std::string& name) 
			: m_name(name) 
		{
//			alarm_handle += igris::make_delegate(&device::alarm_handle_debug, this);
//			ready_handle += igris::make_delegate(&device::ready_handle_debug, this);
		}

		std::string& name() { return m_name; }
		const std::string& name() const { return m_name; }

		void add_depend_device(device * dev);
		void alarm(std::string data);
		void allgood();

		void increment_supervisors_alarm_counter();
		void decrement_supervisors_alarm_counter();

		const std::string& status_string() { return m_status_string; }

		virtual void alarm_handle() 
		{ 
			dprln("virtdev::ALARM", name().c_str()); 
		}
		
		virtual void ready_handle() 
		{ 
			dprln("virtdev::READY", name().c_str()); 
		}

		void update_alarm_message(const std::string& str) 
		{
			m_status_string = str;
			alarm_handle();
		}
	};
}

#endif