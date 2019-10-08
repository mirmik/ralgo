#include "device.h"

#include <assert.h>
#include <mutex>

static std::recursive_mutex mtx;

std::string collect_status() 
{
	return "StatusCollectAlgoToDo";
}

void virtdevs::device::increment_supervisors_alarm_counter() 
{
	std::lock_guard<std::recursive_mutex> lock(mtx);
	for (auto s : supervisors) 
	{
		s->alarmed_deps++;
		assert(s->alarmed_deps < (int)s->supervisors.size());
		s->increment_supervisors_alarm_counter();

		s->alarm_handle();
	}
}

void virtdevs::device::decrement_supervisors_alarm_counter() 
{
	std::lock_guard<std::recursive_mutex> lock(mtx);
	for (auto s : supervisors) 
	{
		s->alarmed_deps--;
		assert(s->alarmed_deps >= 0);
		s->decrement_supervisors_alarm_counter();

		if (s->alarmed_deps == 0) 
		{
			s->ready_handle();
		}
	}
}

void virtdevs::device::add_depend_device(device * dev)
{
	std::lock_guard<std::recursive_mutex> lock(mtx);

	depends.push_back(dev);
	dev->supervisors.push_back(this);

	if (dev->status == Status::Ok)
	{}
	else
		alarmed_deps++;
}

void virtdevs::device::alarm(std::string data)
{
	std::lock_guard<std::recursive_mutex> lock(mtx);

	if (status == Status::Alarm) 
	{
		if (m_status_string != data) 
			update_alarm_message(data);
		return;
	}

	else
	{
		status = Status::Alarm;
		m_status_string = data;
		increment_supervisors_alarm_counter();
	}
}

void virtdevs::device::allgood()
{
	std::lock_guard<std::recursive_mutex> lock(mtx);

	if (status == Status::Ok)
		return;

	else 
	{
		status = Status::Ok;
		m_status_string = "";
		decrement_supervisors_alarm_counter();
	}
}