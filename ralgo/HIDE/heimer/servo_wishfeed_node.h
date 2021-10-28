/** @file */

#ifndef RALGO_HEIMER_SERVO_WISHFEED_NODE_H
#define RALGO_HEIMER_SERVO_WISHFEED_NODE_H

#include <igris/container/array_view.h>
#include <ralgo/heimer/datanode.h>
#include <ralgo/heimer/node.h>
#include <ralgo/heimer/servo_wishfeed.h>
#include <ralgo/heimer/signal.h>

#define SERVO_WISHFEED_MAXDIM 3

#define FEED_PHASE 0
#define WISH_PHASE 1

namespace heimer
{
    class servo_wishfeed_node : public heimer::node
    {
        using parent = heimer::node;

    protected:
        int _left_dim;
        int _right_dim;

        std::array<datanode_ptr, SERVO_WISHFEED_MAXDIM> _left_signals;
        std::array<datanode_ptr, SERVO_WISHFEED_MAXDIM> _right_signals;

    public:
        int init(const char *name, const igris::array_view<datanode_ptr> &lsigs,
                 const igris::array_view<datanode_ptr> &rsigs);

        virtual void feed_doit() = 0;
        virtual void wish_doit() = 0;

        void setup_doit();
        void shutdown_doit();

        int doit(NodeMode mode, int argc, const int *argv) override;

        heimer::errcode command(int argc, const char **argv) override
        {
            heimer::errcode sts;

            return parent::command(argc, argv);
        }
    };

    class servo_wishfeed_transnode : public servo_wishfeed_node
    {
        using parent = servo_wishfeed_node;

    public:
        virtual void provide_feed_signal(real servo_wishfeed::*fieldptr) = 0;
        virtual void provide_wish_signal(real servo_wishfeed::*fieldptr) = 0;

        void feed_doit() override
        {
            provide_feed_signal(&servo_wishfeed::feedctr);
            provide_feed_signal(&servo_wishfeed::feedpos);
        }

        void wish_doit() override
        {
            provide_wish_signal(&servo_wishfeed::wishpos);
            provide_wish_signal(&servo_wishfeed::wishspd);
        }

        heimer::errcode command(int argc, const char **argv) override
        {
            heimer::errcode sts;

            return parent::command(argc, argv);
        }
    };
}

#endif
