#ifndef RALGO_HEIMER_ERRCODE_H
#define RALGO_HEIMER_ERRCODE_H

namespace heimer
{
	struct errcode
	{
		enum
		{
			OK = 0,
			DATANODE_IS_USED_ERROR,
			DATANODE_NOT_FOUND_ERROR,
			UNRESOLVED_COMMAND
		} value;

		errcode() : value(OK) {}
		errcode(decltype(value) value) { this->value = value; }

		operator bool () { return (int) value; }

		const char * to_string() 
		{
			switch (value) 
			{
				case OK: return "noerror";
				case DATANODE_IS_USED_ERROR: return "DataNode is busy";
				case DATANODE_NOT_FOUND_ERROR: return "DataNode is not found";
				default: return "unresolved error code";
			}
		} 
	};
}

#endif