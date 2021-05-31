#include <ralgo/heimer/node.h>
#include <string.h>

int heimer::node::name_compare(const char * name) 
{
	return strncmp(_name, name, NODE_NAME_MAXLEN);
}

void heimer::node::rename(const char * name) 
{
	strncpy(_name, name, NODE_NAME_MAXLEN);	
}