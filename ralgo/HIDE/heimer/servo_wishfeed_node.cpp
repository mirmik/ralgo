/** @file */

#include <ralgo/heimer/servo_wishfeed_node.h>

int heimer::servo_wishfeed_node::init(
    const char *name, const igris::array_view<datanode_ptr> &lsigs,
    const igris::array_view<datanode_ptr> &rsigs)
{
    rename(name);

    std::copy(lsigs.begin(), lsigs.end(), _left_signals.begin());
    std::copy(rsigs.begin(), rsigs.end(), _right_signals.begin());

    _left_dim = _left_signals.size();
    _right_dim = _right_signals.size();

    return 0;
}

int heimer::servo_wishfeed_node::doit(NodeMode mode, int argc, const int *argv)
{
    (void)argc;

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
            heimer::send_signal(HEIMER_SIGNAL_WRONG_NODE_MODE, (void *)this);
            return -1;
        }
    }
    }

    return -1;
}

void heimer::servo_wishfeed_node::setup_doit() {}

void heimer::servo_wishfeed_node::shutdown_doit() {}
