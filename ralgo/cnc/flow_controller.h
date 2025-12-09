#ifndef RALGO_CNC_FLOW_CONTROLLER_H
#define RALGO_CNC_FLOW_CONTROLLER_H

#include <igris/event/delegate.h>
#include <cstddef>
#include <cstdint>
#include <ralgo/cnc/defs.h>

namespace cnc
{
    /**
     * Events that flow_controller can report to the host.
     */
    enum class flow_event : uint8_t
    {
        // Command acknowledged, queue has room
        // Context: available room in queue
        command_ok = 0,

        // Command rejected - queue is full
        // Context: 0
        queue_full = 1,

        // Motion completed (all blocks executed)
        // Context: 0
        motion_complete = 2,

        // Queue state changed (block started/finished)
        // Context: blocks in queue
        queue_changed = 3,
    };

    /**
     * Flow control information sent to host.
     */
    struct flow_status
    {
        flow_event event = flow_event::command_ok;
        uint8_t queue_size = 0;      ///< Total queue capacity
        uint8_t queue_used = 0;      ///< Blocks currently in queue
        uint8_t queue_room = 0;      ///< Available room for new blocks
        int32_t last_blockno = -1;   ///< Last completed block number
    };

    /**
     * Flow controller for streaming protocol.
     *
     * Manages communication between CNC controller and host for:
     * - Acknowledging received commands
     * - Reporting queue state for flow control
     * - Notifying motion completion
     *
     * Design:
     * - Host sends commands without waiting for individual acks
     * - Host tracks how many unacknowledged commands are in flight
     * - Controller reports queue state so host knows when to send more
     *
     * Usage:
     *   cnc::flow_controller flow;
     *   flow.set_callback([](const flow_status& s) {
     *       send_to_host(format("ok Q{}", s.queue_room));
     *   });
     *
     *   // After adding block:
     *   flow.report_ok(queue_room);
     *
     *   // When queue is full:
     *   flow.report_full();
     *
     *   // When motion completes:
     *   flow.report_complete();
     */
    class flow_controller
    {
    public:
        using callback_t = igris::delegate<void, const flow_status &>;

    private:
        callback_t _callback = {};
        uint8_t _queue_capacity = 0;
        int32_t _last_completed_blockno = -1;
        bool _enabled = true;

    public:
        flow_controller() = default;

        // Non-copyable
        flow_controller(const flow_controller &) = delete;
        flow_controller &operator=(const flow_controller &) = delete;

        /**
         * Set callback to be called on flow events.
         * Callback receives flow_status with event details.
         */
        void set_callback(callback_t cb)
        {
            _callback = cb;
        }

        /**
         * Set queue capacity for status reporting.
         */
        void set_queue_capacity(uint8_t capacity)
        {
            _queue_capacity = capacity;
        }

        /**
         * Enable/disable flow control notifications.
         */
        void set_enabled(bool en)
        {
            _enabled = en;
        }

        bool is_enabled() const
        {
            return _enabled;
        }

        /**
         * Report successful command processing.
         * @param queue_used Current blocks in queue
         * @param queue_room Available room in queue
         * @param blockno Block number that was added
         */
        void report_ok(uint8_t queue_used, uint8_t queue_room, int32_t blockno = -1)
        {
            if (!_enabled || !_callback)
                return;

            flow_status status;
            status.event = flow_event::command_ok;
            status.queue_size = _queue_capacity;
            status.queue_used = queue_used;
            status.queue_room = queue_room;
            status.last_blockno = blockno;

            _callback(status);
        }

        /**
         * Report queue full - command rejected.
         * @param queue_used Current blocks in queue (should equal capacity)
         */
        void report_full(uint8_t queue_used)
        {
            if (!_callback)
                return;

            flow_status status;
            status.event = flow_event::queue_full;
            status.queue_size = _queue_capacity;
            status.queue_used = queue_used;
            status.queue_room = 0;
            status.last_blockno = _last_completed_blockno;

            _callback(status);
        }

        /**
         * Report motion complete - all blocks executed, system idle.
         */
        void report_complete()
        {
            if (!_callback)
                return;

            flow_status status;
            status.event = flow_event::motion_complete;
            status.queue_size = _queue_capacity;
            status.queue_used = 0;
            status.queue_room = _queue_capacity;
            status.last_blockno = _last_completed_blockno;

            _callback(status);
        }

        /**
         * Report queue state change (block completed).
         * @param queue_used Current blocks in queue
         * @param queue_room Available room
         * @param completed_blockno Block that was completed
         */
        void report_block_complete(uint8_t queue_used,
                                   uint8_t queue_room,
                                   int32_t completed_blockno)
        {
            _last_completed_blockno = completed_blockno;

            if (!_enabled || !_callback)
                return;

            flow_status status;
            status.event = flow_event::queue_changed;
            status.queue_size = _queue_capacity;
            status.queue_used = queue_used;
            status.queue_room = queue_room;
            status.last_blockno = completed_blockno;

            _callback(status);
        }

        /**
         * Get last completed block number.
         */
        int32_t last_completed_blockno() const
        {
            return _last_completed_blockno;
        }

        /**
         * Get event name as string.
         */
        static const char *event_name(flow_event event)
        {
            switch (event)
            {
            case flow_event::command_ok:
                return "ok";
            case flow_event::queue_full:
                return "full";
            case flow_event::motion_complete:
                return "complete";
            case flow_event::queue_changed:
                return "changed";
            default:
                return "unknown";
            }
        }
    };
}

#endif
