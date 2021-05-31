#ifndef RALGO_HEIMER_DATANODE_H
#define RALGO_HEIMER_DATANODE_H

#include <ralgo/heimer/servo_wishfeed.h>

#define DATANODE_TYPEHINT_SERVOWISHFEED 1
#define DATANODE_TYPEHINT_UNDEFINED -1
#define DATANODE_NAME_MAXLEN 16

namespace heimer 
{
	class datanode 
	{
		char _name [DATANODE_NAME_MAXLEN];
		int _typehint;

		union 
		{
			real raw[0];
			servo_wishfeed servowf;
		} data;

	public:
		static int typehint_cast(const char * strhint) 
		{
			if (strcmp(strhint, "servowf") == 0) return DATANODE_TYPEHINT_SERVOWISHFEED;

			return DATANODE_TYPEHINT_UNDEFINED;
		}
		\
		datanode() {}

		servo_wishfeed & as_servowf() { return data.servowf; }

		datanode(const char * name, const char * hint) 
		{
			init(name, hint);
		}

		datanode(const char * name, int hint) 
		{
			init(name, hint);
		}

		void init(const char * name, const char * hint) 
		{
			init(name, typehint_cast(hint));
		}

		void init(const char * name, int hint) 
		{
			strncpy(_name, name, DATANODE_NAME_MAXLEN);
			_typehint = hint;
			clean();
		}

		void clean() 
		{
			memset(&data, 0, sizeof(data));
		}

		int typehint() 
		{
			return _typehint;
		}

		const char * name() 
		{
			return _name;
		}

		int compare_name(const char * name) 
		{
			return strncmp(_name, name, DATANODE_NAME_MAXLEN);
		}
	};
}

#endif