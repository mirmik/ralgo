/** @file */

#ifndef RALGO_HEIMER_DATANODE_H
#define RALGO_HEIMER_DATANODE_H

#include <ralgo/heimer/servo_wishfeed.h>

#define DATANODE_TYPEHINT_SERVOWISHFEED 1
#define DATANODE_TYPEHINT_UNDEFINED -1
#define DATANODE_NAME_MAXLEN 16

namespace heimer 
{
	class datanode_ptr;

	class datanode 
	{
		char _name [DATANODE_NAME_MAXLEN];
		int _typehint;
		int _refs = 0;

		union 
		{
			real raw[0];
			servo_wishfeed servowf;
		} data;

	public:
	    bool is_used() 
	    {
	    	return _refs != 0; 
	    }

		static int typehint_cast(const char * strhint) 
		{
			if (strcmp(strhint, "servowf") == 0) return DATANODE_TYPEHINT_SERVOWISHFEED;

			return DATANODE_TYPEHINT_UNDEFINED;
		}

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

		int typehint() const
		{
			return _typehint;
		}

		const char * name() const
		{
			return _name;
		}

		int compare_name(const char * name) const
		{
			return strncmp(_name, name, DATANODE_NAME_MAXLEN);
		}

		friend datanode_ptr;
	};

	class datanode_ptr 
	{
		datanode * ptr;

	public:
		datanode_ptr() : ptr(nullptr) {}
		datanode_ptr(datanode * dn) : ptr(dn) { ++ptr->_refs; }
		~datanode_ptr() { release(); }

		datanode * operator *  () { return ptr; }
		datanode * operator -> () { return ptr; }
		
		const datanode * operator *  () const { return ptr; }
		const datanode * operator -> () const { return ptr; }

	    datanode_ptr & operator = (datanode_ptr && dn) 
	    { 
	    	release();

	    	ptr = dn.ptr; 
	    	dn.ptr = 0;
	    	return *this; 
	    }

	    datanode_ptr & operator = (const datanode_ptr & dn) 
	    { 
	    	release();

	    	ptr = dn.ptr; 
	    	++ptr->_refs;
	    	return *this; 
	    }

	    void release() 
	    {
			if (ptr) 
			{
				assert(ptr->_refs > 0);
	    		--ptr->_refs;
			}

	    	ptr = nullptr;
	    }
	};
}

#endif