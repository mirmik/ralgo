#include <doctest/doctest.h>
#include <ralgo/cnc/flow_controller.h>
#include <cstring>

// Helper class for receiving flow callbacks in tests
class flow_test_receiver
{
public:
    cnc::flow_status last_status = {};
    int call_count = 0;

    void on_flow_event(const cnc::flow_status &s)
    {
        last_status = s;
        call_count++;
    }

    void reset()
    {
        last_status = {};
        call_count = 0;
    }
};

TEST_CASE("flow_controller.basic")
{
    cnc::flow_controller flow;

    SUBCASE("default state")
    {
        CHECK(flow.is_enabled());
        CHECK_EQ(flow.last_completed_blockno(), -1);
    }

    SUBCASE("enable/disable")
    {
        flow.set_enabled(false);
        CHECK_EQ(flow.is_enabled(), false);

        flow.set_enabled(true);
        CHECK(flow.is_enabled());
    }
}

TEST_CASE("flow_controller.callbacks")
{
    cnc::flow_controller flow;
    flow.set_queue_capacity(10);

    flow_test_receiver receiver;
    flow.set_callback(
        igris::make_delegate(&flow_test_receiver::on_flow_event, &receiver));

    SUBCASE("report_ok sends callback")
    {
        flow.report_ok(3, 7, 42);

        CHECK_EQ(receiver.call_count, 1);
        CHECK_EQ(receiver.last_status.event, cnc::flow_event::command_ok);
        CHECK_EQ(receiver.last_status.queue_size, 10);
        CHECK_EQ(receiver.last_status.queue_used, 3);
        CHECK_EQ(receiver.last_status.queue_room, 7);
        CHECK_EQ(receiver.last_status.last_blockno, 42);
    }

    SUBCASE("report_full sends callback")
    {
        flow.report_full(10);

        CHECK_EQ(receiver.call_count, 1);
        CHECK_EQ(receiver.last_status.event, cnc::flow_event::queue_full);
        CHECK_EQ(receiver.last_status.queue_used, 10);
        CHECK_EQ(receiver.last_status.queue_room, 0);
    }

    SUBCASE("report_complete sends callback")
    {
        flow.report_complete();

        CHECK_EQ(receiver.call_count, 1);
        CHECK_EQ(receiver.last_status.event, cnc::flow_event::motion_complete);
        CHECK_EQ(receiver.last_status.queue_used, 0);
        CHECK_EQ(receiver.last_status.queue_room, 10);
    }

    SUBCASE("report_block_complete sends callback and updates last_blockno")
    {
        flow.report_block_complete(5, 5, 100);

        CHECK_EQ(receiver.call_count, 1);
        CHECK_EQ(receiver.last_status.event, cnc::flow_event::queue_changed);
        CHECK_EQ(receiver.last_status.queue_used, 5);
        CHECK_EQ(receiver.last_status.queue_room, 5);
        CHECK_EQ(receiver.last_status.last_blockno, 100);
        CHECK_EQ(flow.last_completed_blockno(), 100);
    }

    SUBCASE("disabled flow does not send ok callbacks")
    {
        flow.set_enabled(false);

        flow.report_ok(3, 7, 42);
        CHECK_EQ(receiver.call_count, 0);

        flow.report_block_complete(5, 5, 100);
        CHECK_EQ(receiver.call_count, 0);
    }

    SUBCASE("report_full sends even when disabled")
    {
        flow.set_enabled(false);

        // queue_full is always reported (critical event)
        flow.report_full(10);

        CHECK_EQ(receiver.call_count, 1);
        CHECK_EQ(receiver.last_status.event, cnc::flow_event::queue_full);
    }

    SUBCASE("report_complete sends even when disabled")
    {
        flow.set_enabled(false);

        // motion_complete is always reported
        flow.report_complete();

        CHECK_EQ(receiver.call_count, 1);
        CHECK_EQ(receiver.last_status.event, cnc::flow_event::motion_complete);
    }
}

TEST_CASE("flow_controller.event_names")
{
    CHECK(strcmp(cnc::flow_controller::event_name(cnc::flow_event::command_ok), "ok") == 0);
    CHECK(strcmp(cnc::flow_controller::event_name(cnc::flow_event::queue_full), "full") == 0);
    CHECK(strcmp(cnc::flow_controller::event_name(cnc::flow_event::motion_complete), "complete") == 0);
    CHECK(strcmp(cnc::flow_controller::event_name(cnc::flow_event::queue_changed), "changed") == 0);
}
