#ifndef RALGO_CNC_BUFFER_CONTROLLER_H
#define RALGO_CNC_BUFFER_CONTROLLER_H

/**
 * @file buffer_controller.h
 * @brief Buffering control for look-ahead optimization.
 *
 * Buffer controller manages command buffering to enable effective
 * look-ahead planning. It supports two modes:
 *
 * 1. Automatic buffering: waits for N blocks OR timeout before starting
 *    motion. This allows look-ahead to optimize the first blocks.
 *
 * 2. Explicit buffering: host explicitly starts buffering with buffer_enable(),
 *    sends commands, then calls buffer_start() to execute.
 *
 * Usage (automatic):
 *   buffer.set_min_blocks_to_start(3);
 *   buffer.set_timeout_ticks(1000);  // 100ms at 10kHz
 *
 * Usage (explicit):
 *   buffer.enable_explicit();
 *   // ... send commands ...
 *   buffer.start_explicit();
 */

#include <cstdint>

namespace cnc
{
    /**
     * Buffer controller for motion command buffering.
     */
    class buffer_controller
    {
    private:
        // Minimum blocks to accumulate before starting (0 = no auto-buffering)
        int _min_blocks_to_start = 0;

        // Timeout in ticks (0 = no timeout)
        int64_t _timeout_ticks = 0;

        // Tick when first block was added to empty queue
        int64_t _start_tick = 0;

        // Explicit buffer mode active
        bool _explicit_mode = false;

        // Frequency for ms/tick conversion
        double _frequency = 0;

    public:
        buffer_controller() = default;

        /**
         * Set revolver frequency for time conversions.
         */
        void set_frequency(double freq)
        {
            _frequency = freq;
        }

        /**
         * Set minimum blocks to accumulate before starting.
         * @param n Number of blocks (0 = disable auto-buffering)
         */
        void set_min_blocks_to_start(int n)
        {
            _min_blocks_to_start = n;
        }

        int min_blocks_to_start() const
        {
            return _min_blocks_to_start;
        }

        /**
         * Set timeout in milliseconds.
         * After timeout, motion starts even with fewer blocks.
         * @param ms Timeout in milliseconds (0 = no timeout)
         */
        void set_timeout_ms(int ms)
        {
            if (_frequency > 0)
                _timeout_ticks = static_cast<int64_t>(ms * _frequency / 1000);
            else
                _timeout_ticks = 0;
        }

        int timeout_ms() const
        {
            if (_frequency > 0)
                return static_cast<int>(_timeout_ticks * 1000 / _frequency);
            return 0;
        }

        /**
         * Set timeout in ticks directly.
         */
        void set_timeout_ticks(int64_t ticks)
        {
            _timeout_ticks = ticks;
        }

        int64_t timeout_ticks() const
        {
            return _timeout_ticks;
        }

        /**
         * Record the start tick when first block is added.
         * Call this when adding first block to empty queue.
         */
        void record_start_tick(int64_t tick)
        {
            _start_tick = tick;
        }

        /**
         * Enable explicit buffering mode.
         * Commands are accumulated but not executed until start_explicit().
         */
        void enable_explicit()
        {
            _explicit_mode = true;
        }

        /**
         * Start execution of explicitly buffered commands.
         * Returns true if was in explicit mode.
         */
        bool start_explicit()
        {
            if (_explicit_mode)
            {
                _explicit_mode = false;
                return true;
            }
            return false;
        }

        /**
         * Cancel explicit buffering mode.
         */
        void cancel_explicit()
        {
            _explicit_mode = false;
        }

        /**
         * Check if explicit buffer mode is active.
         */
        bool is_explicit_mode() const
        {
            return _explicit_mode;
        }

        /**
         * Check if buffer is ready to start motion.
         *
         * @param pending_blocks Number of blocks waiting in queue
         * @param current_tick Current iteration counter
         * @param is_active Whether motion is already active
         * @return true if motion can start
         */
        bool is_ready(int pending_blocks, int64_t current_tick, bool is_active) const
        {
            // Explicit mode - manual control
            if (_explicit_mode)
                return false;

            // No auto-buffering configured - always ready
            if (_min_blocks_to_start <= 0)
                return true;

            // Already in motion - ready
            if (is_active)
                return true;

            // Enough blocks accumulated?
            if (pending_blocks >= _min_blocks_to_start)
                return true;

            // Timeout expired?
            if (_timeout_ticks > 0 && pending_blocks > 0)
            {
                if (current_tick - _start_tick >= _timeout_ticks)
                    return true;
            }

            return false;
        }

        /**
         * Check if auto-buffering should pause planner on first block.
         */
        bool should_pause_on_first_block() const
        {
            return _min_blocks_to_start > 0 && !_explicit_mode;
        }
    };
}

#endif
