/** @file */

#ifndef RALGO_HEIMER_STATUS_H
#define RALGO_HEIMER_STATUS_H

namespace heimer
{
	enum class Status : int
	{
		OK = 0,
		UNRESOLVED_SIGNAL,
		SIGNAL_CONNECTION_CONFLICT
	};

	struct StatusPair 
	{
		Status status;
		std::string_view note;
	};
}

#endif
