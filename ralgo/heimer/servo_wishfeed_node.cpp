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

	return 0;
}

int heimer::servo_wishfeed_node::doit(NodeMode mode, int argc, const int * argv) 
{
	(void) argc;

	switch (mode) 
	{
		case NodeMode::SETUP: 
		{
			setup_doit();
			return 0;
		}

		case NodeMode::SHUTDOWN: 
		{
			shutdown_doit();
			return 0;
		}

		case NodeMode::NORMAL:
		{
			switch (argv[0]) 
			{
				case FEED_PHASE:
					feed_doit();
					return 0;

				case WISH_PHASE:
					wish_doit();
					return 0;

				default:
					heimer::send_signal(HEIMER_SIGNAL_WRONG_NODE_MODE, (void*) this);
					return -1;
			}		
		}
	}

	return -1;
}

void heimer::servo_wishfeed_node::setup_doit() 
{

}

void heimer::servo_wishfeed_node::shutdown_doit() 
{
	
}