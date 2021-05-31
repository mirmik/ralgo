#include <ralgo/heimer/servo_wishfeed_node.h>

int heimer::servo_wishfeed_node::init(
	const char * name,
    const igris::array_view<datanode *> & lsigs,
    const igris::array_view<datanode *> & rsigs
) 
{
	rename(name);

	for (int i = 0; i < lsigs.size(); ++i) 
	{
		if (lsigs[i]->typehint() == DATANODE_TYPEHINT_SERVOWISHFEED)
			_left_signals[i] = &lsigs[i]->as_servowf();
		else 
			return -1;
	}

	for (int i = 0; i < rsigs.size(); ++i) 
	{
		if (rsigs[i]->typehint() == DATANODE_TYPEHINT_SERVOWISHFEED)
			_right_signals[i] = &rsigs[i]->as_servowf();
		else 
			return -1;
	}
}