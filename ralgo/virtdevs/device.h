#ifndef VIRTDEVS_DEVICE_H
#define VIRTDEVS_DEVICE_H

#include <vector>
#include <string>

extern "C" void debug_print(const char*);

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
		std::string status_string;

		std::vector<device *> supervisors;
		std::vector<device *> depends;
		
		int alarmed_deps = 0;

		std::string m_name;
		
	protected:
		void recursive_alarm(device * source, std::string text);
		void recursive_allgood();

	public:
		device(const std::string& name) : m_name(name) {}

		std::string& name() { return m_name; }
		const std::string& name() const { return m_name; }

		void add_depend_device(device * dev);
		void alarm(std::string data);
		void allgood();

		void increment_supervisors_alarm_counter();
		void decrement_supervisors_alarm_counter();

		void alarm_handle() { debug_print("ALARM\n"); }
		void ready_handle() { debug_print("READY\n"); }
	};
}

#endif