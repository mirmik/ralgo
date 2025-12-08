#ifndef RALGO_CNC_ERROR_HANDLER_H
#define RALGO_CNC_ERROR_HANDLER_H

#include <array>
#include <atomic>
#include <cstdint>
#include <igris/event/delegate.h>
#include <igris/sync/syslock.h>
#include <ralgo/cnc/defs.h>

namespace cnc
{
    /**
     * Error codes for CNC subsystem.
     * Grouped by severity and source.
     */
    enum class error_code : uint16_t
    {
        // === No error ===
        ok = 0,

        // === Configuration errors (100-199) ===
        invalid_frequency = 100,      ///< revolver_frequency <= 0
        invalid_steps_per_mm = 101,   ///< steps_per_mm <= 0 for axis
        invalid_axes_count = 102,     ///< axes count out of range
        not_initialized = 103,        ///< system not properly initialized

        // === Motion errors (200-299) ===
        invalid_velocity = 200,       ///< velocity <= 0
        invalid_acceleration = 201,   ///< acceleration <= 0
        invalid_distance = 202,       ///< zero or negative distance
        invalid_direction = 203,      ///< direction vector is zero
        block_validation_failed = 204,///< planner_block validation failed
        queue_overflow = 205,         ///< block queue is full

        // === Runtime errors (300-399) ===
        position_limit_exceeded = 300,///< soft limit hit
        following_error = 301,        ///< feedback error too large
        emergency_stop = 302,         ///< e-stop triggered
        watchdog_timeout = 303,       ///< watchdog fired

        // === Hardware errors (400-499) ===
        stepper_fault = 400,          ///< stepper driver fault
        encoder_error = 401,          ///< encoder read error
        communication_error = 402,    ///< host communication lost
    };

    /**
     * Error entry with context information.
     */
    struct error_entry
    {
        error_code code = error_code::ok;
        int8_t axis = -1;             ///< Axis index (-1 if not axis-specific)
        uint32_t timestamp = 0;       ///< System tick when error occurred
        int32_t context = 0;          ///< Additional context (e.g. actual value)
    };

    /**
     * Lock-free error collector for real-time systems.
     *
     * Design:
     * - Errors are stored in a ring buffer
     * - Writing is lock-free (atomic index)
     * - Reading should happen from a single consumer thread
     * - Callback notifies when new error is added
     *
     * Usage:
     *   cnc::error_handler errors;
     *   errors.set_callback([](){ notify_host(); });
     *
     *   // In real-time code:
     *   errors.report(error_code::invalid_velocity, axis, value);
     *
     *   // In background thread:
     *   error_entry e;
     *   while (errors.pop(e)) {
     *       send_to_host(e);
     *   }
     */
    class error_handler
    {
    public:
        static constexpr size_t BUFFER_SIZE = 32;

    private:
        std::array<error_entry, BUFFER_SIZE> _buffer = {};
        std::atomic<uint32_t> _head{0};  ///< Next write position
        std::atomic<uint32_t> _tail{0};  ///< Next read position
        std::atomic<uint32_t> _tick{0};  ///< Timestamp counter
        igris::delegate<void> _callback = {};
        std::atomic<uint32_t> _total_errors{0};
        std::atomic<uint32_t> _dropped_errors{0};

    public:
        error_handler() = default;

        // Non-copyable
        error_handler(const error_handler&) = delete;
        error_handler& operator=(const error_handler&) = delete;

        /**
         * Set callback to be called when error is reported.
         * Callback should be fast and non-blocking (just set a flag).
         */
        void set_callback(igris::delegate<void> cb)
        {
            _callback = cb;
        }

        /**
         * Report an error. Safe to call from ISR.
         * @param code Error code
         * @param axis Axis index (-1 if not axis-specific)
         * @param context Additional context value
         */
        void report(error_code code, int8_t axis = -1, int32_t context = 0)
        {
            uint32_t head = _head.load(std::memory_order_relaxed);
            uint32_t next = (head + 1) % BUFFER_SIZE;
            uint32_t tail = _tail.load(std::memory_order_acquire);

            if (next == tail)
            {
                // Buffer full - drop error
                _dropped_errors.fetch_add(1, std::memory_order_relaxed);
                return;
            }

            _buffer[head] = {
                code,
                axis,
                _tick.load(std::memory_order_relaxed),
                context
            };

            _head.store(next, std::memory_order_release);
            _total_errors.fetch_add(1, std::memory_order_relaxed);

            if (_callback)
                _callback();
        }

        /**
         * Pop an error from the buffer. Call from consumer thread only.
         * @param out Output error entry
         * @return true if error was retrieved, false if buffer empty
         */
        bool pop(error_entry& out)
        {
            uint32_t tail = _tail.load(std::memory_order_relaxed);
            uint32_t head = _head.load(std::memory_order_acquire);

            if (tail == head)
                return false;

            out = _buffer[tail];
            _tail.store((tail + 1) % BUFFER_SIZE, std::memory_order_release);
            return true;
        }

        /**
         * Check if there are pending errors.
         */
        bool has_errors() const
        {
            return _head.load(std::memory_order_acquire) !=
                   _tail.load(std::memory_order_acquire);
        }

        /**
         * Get count of pending errors.
         */
        size_t pending_count() const
        {
            uint32_t head = _head.load(std::memory_order_acquire);
            uint32_t tail = _tail.load(std::memory_order_acquire);
            return (head - tail + BUFFER_SIZE) % BUFFER_SIZE;
        }

        /**
         * Get total errors reported since start.
         */
        uint32_t total_errors() const
        {
            return _total_errors.load(std::memory_order_relaxed);
        }

        /**
         * Get count of dropped errors (buffer overflow).
         */
        uint32_t dropped_errors() const
        {
            return _dropped_errors.load(std::memory_order_relaxed);
        }

        /**
         * Increment timestamp. Call from system tick.
         */
        void tick()
        {
            _tick.fetch_add(1, std::memory_order_relaxed);
        }

        /**
         * Clear all errors.
         */
        void clear()
        {
            _head.store(0, std::memory_order_release);
            _tail.store(0, std::memory_order_release);
        }

        /**
         * Get error name as string.
         */
        static const char* error_name(error_code code)
        {
            switch (code)
            {
            case error_code::ok: return "ok";
            case error_code::invalid_frequency: return "invalid_frequency";
            case error_code::invalid_steps_per_mm: return "invalid_steps_per_mm";
            case error_code::invalid_axes_count: return "invalid_axes_count";
            case error_code::not_initialized: return "not_initialized";
            case error_code::invalid_velocity: return "invalid_velocity";
            case error_code::invalid_acceleration: return "invalid_acceleration";
            case error_code::invalid_distance: return "invalid_distance";
            case error_code::invalid_direction: return "invalid_direction";
            case error_code::block_validation_failed: return "block_validation_failed";
            case error_code::queue_overflow: return "queue_overflow";
            case error_code::position_limit_exceeded: return "position_limit_exceeded";
            case error_code::following_error: return "following_error";
            case error_code::emergency_stop: return "emergency_stop";
            case error_code::watchdog_timeout: return "watchdog_timeout";
            case error_code::stepper_fault: return "stepper_fault";
            case error_code::encoder_error: return "encoder_error";
            case error_code::communication_error: return "communication_error";
            default: return "unknown";
            }
        }
    };
}

#endif
