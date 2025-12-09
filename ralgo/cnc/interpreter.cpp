#include <ralgo/cnc/interpreter.h>

namespace cnc
{
    // === CLI command handlers ===

    int interpreter::cmd_setprotect(const nos::argv &, nos::ostream &)
    {
        ralgo::global_protection = false;
        ralgo::info("Protection disabled");
        return 0;
    }

    int interpreter::cmd_stop(const nos::argv &, nos::ostream &)
    {
        smooth_stop();
        return 0;
    }

    int interpreter::cmd_lastblock(const nos::argv &, nos::ostream &os)
    {
        last_block().print_to_stream(os);
        return 0;
    }

    int interpreter::cmd_relmove(const nos::argv &argv, nos::ostream &os)
    {
        if (argv.size() < 2)
        {
            nos::println_to(os, "Usage: relmove <axis><dist>... F<feed_mm/s> M<accel_mm/s2>");
            nos::println_to(os, "  Axes: X,Y,Z,A,B,C,I,J,K (0-8)");
            nos::println_to(os, "  Example: relmove X10 Y-5.5 Z2 F100 M500");
            return 0;
        }
        command_incremental_move(argv.without(1), os);
        return 0;
    }

    int interpreter::cmd_absmove(const nos::argv &argv, nos::ostream &os)
    {
        if (argv.size() < 2)
        {
            nos::println_to(os, "Usage: absmove <axis><pos>... F<feed_mm/s> M<accel_mm/s2>");
            nos::println_to(os, "  Axes: X,Y,Z,A,B,C,I,J,K (0-8)");
            nos::println_to(os, "  Example: absmove X100 Y50 Z0 F100 M500");
            return 0;
        }
        command_absolute_move(argv.without(1), os);
        return 0;
    }

    int interpreter::cmd_abspulses(const nos::argv &argv, nos::ostream &os)
    {
        if (argv.size() < 2)
        {
            nos::println_to(os, "Usage: abspulses <axis><steps>... F<feed> M<accel>");
            nos::println_to(os, "  Example: abspulses X1000 Y500 F100 M500");
            return 0;
        }
        command_absolute_pulses(argv.without(1), os);
        return 0;
    }

    int interpreter::cmd_steps(const nos::argv &, nos::ostream &os)
    {
        nos::println_to(os, current_steps());
        return 0;
    }

    int interpreter::cmd_finishes(const nos::argv &, nos::ostream &os)
    {
        nos::print_list_to(os, _final_position);
        nos::println_to(os);
        return 0;
    }

    int interpreter::cmd_gains(const nos::argv &, nos::ostream &os)
    {
        nos::print_list_to(os, gains);
        nos::println_to(os);
        return 0;
    }

    int interpreter::cmd_gears(const nos::argv &, nos::ostream &os)
    {
        nos::print_list_to(os, _steps_per_unit);
        nos::println_to(os);
        return 0;
    }

    int interpreter::cmd_setgear(const nos::argv &argv, nos::ostream &os)
    {
        if (argv.size() < 3)
        {
            nos::println_to(os, "Usage: setgear <axis> <steps_per_unit>");
            nos::println_to(os, "  Example: setgear X 100.5");
            return 0;
        }
        auto axno = symbol_to_index(argv[1][0]);
        cnc_float_type val = igris_atof64(argv[2].data(), NULL);
        set_steps_per_unit(axno, val);
        feedback_guard->set_control_to_drive_multiplier(axno, val);
        return 0;
    }

    int interpreter::cmd_set_control_gear(const nos::argv &argv, nos::ostream &os)
    {
        if (argv.size() < 3)
        {
            nos::println_to(os, "Usage: set_control_gear <axis> <value>");
            return 0;
        }
        auto axno = symbol_to_index(argv[1][0]);
        cnc_float_type val = igris_atof64(argv[2].data(), NULL);
        set_steps_per_unit(axno, val);
        feedback_guard->set_control_to_drive_multiplier(axno, val);
        return 0;
    }

    int interpreter::cmd_set_feedback_gear(const nos::argv &argv, nos::ostream &os)
    {
        if (argv.size() < 3)
        {
            nos::println_to(os, "Usage: set_feedback_gear <axis> <value>");
            return 0;
        }
        auto axno = symbol_to_index(argv[1][0]);
        cnc_float_type val = igris_atof64(argv[2].data(), NULL);
        feedback_guard->set_feedback_to_drive_multiplier(axno, val);
        return 0;
    }

    int interpreter::cmd_setpos(const nos::argv &argv, nos::ostream &os)
    {
        if (argv.size() < 3)
        {
            nos::println_to(os, "Usage: setpos <axis> <position_mm>");
            nos::println_to(os, "  Sets current position without moving (for homing)");
            nos::println_to(os, "  Example: setpos X 0");
            return 0;
        }
        auto axno = symbol_to_index(argv[1][0]);
        cnc_float_type val_mm = igris_atof64(argv[2].data(), NULL);
        system_lock();
        _final_position[axno] = val_mm;
        steps_t steps = static_cast<steps_t>(val_mm * _steps_per_unit[axno]);
        revolver->get_steppers()[axno]->set_counter_value(steps);
        feedback_guard->set_feedback_position(axno, val_mm);
        system_unlock();
        return 0;
    }

    int interpreter::cmd_disable_tandem_protection(const nos::argv &argv, nos::ostream &os)
    {
        if (argv.size() < 2)
        {
            nos::println_to(os, "Usage: disable_tandem_protection <index>");
            return 0;
        }
        auto idx = std::stoi(argv[1]);
        feedback_guard->remove_tandem(idx);
        return 0;
    }

    int interpreter::cmd_enable_tandem_protection(const nos::argv &argv, nos::ostream &os)
    {
        if (argv.size() < 2)
        {
            nos::println_to(os, "Usage: enable_tandem_protection <master>,<slave>:<max_error>");
            nos::println_to(os, "  Example: enable_tandem_protection 0,1:10");
            return 0;
        }
        feedback_guard->add_tandem_command(argv.without(1), os);
        return 0;
    }

    int interpreter::cmd_tandem_info(const nos::argv &, nos::ostream &os)
    {
        const auto &tandems = feedback_guard->tandems();
        if (tandems.size() == 0)
        {
            nos::println_to(os, "Tandem list is empty");
        }
        for (auto &tandem : tandems)
        {
            nos::println_to(os, tandem.info());
        }
        return 0;
    }

    int interpreter::cmd_drop_pulses_allowed(const nos::argv &argv, nos::ostream &os)
    {
        if (argv.size() < 2)
        {
            nos::println_to(os, "Usage: drop_pulses_allowed <axis> [<max_pulses>]");
            nos::println_to(os, "  Without value - prints current setting");
            return 0;
        }
        size_t no = std::stoi(argv[1]);

        if (argv.size() > 2)
        {
            int64_t pulses = std::stoi(argv[2]);
            feedback_guard->set_drop_pulses_allowed(no, pulses);
        }
        else
        {
            nos::println_to(os, feedback_guard->drop_pulses_allowed(no));
        }
        return 0;
    }

    int interpreter::cmd_velmaxs(const nos::argv &argv, nos::ostream &os)
    {
        if (argv.size() < 2)
        {
            nos::println_to(os, "Usage: velmaxs <axis>:<velocity>...");
            nos::println_to(os, "  Example: velmaxs 0:100 1:100 2:50");
            return 0;
        }
        auto fmap = args_to_index_value_map(argv.without(1));
        for (auto &[key, val] : fmap)
        {
            max_axes_velocities[key] = val;
        }
        return 0;
    }

    int interpreter::cmd_accmaxs(const nos::argv &argv, nos::ostream &os)
    {
        if (argv.size() < 2)
        {
            nos::println_to(os, "Usage: accmaxs <axis>:<acceleration>...");
            nos::println_to(os, "  Example: accmaxs 0:1000 1:1000 2:500");
            return 0;
        }
        auto fmap = args_to_index_value_map(argv.without(1));
        for (auto &[key, val] : fmap)
            max_axes_accelerations[key] = val;
        return 0;
    }

    int interpreter::cmd_help(const nos::argv &, nos::ostream &os)
    {
        nos::println_to(os, "=== Text commands ===");
        command_help(os);
        nos::println_to(os, "\n=== G-code commands ===");
        gcode_help(os);
        return 0;
    }

    int interpreter::cmd_help_cmd(const nos::argv &, nos::ostream &os)
    {
        command_help(os);
        return 0;
    }

    int interpreter::cmd_help_cnc(const nos::argv &, nos::ostream &os)
    {
        gcode_help(os);
        return 0;
    }

    int interpreter::cmd_state(const nos::argv &, nos::ostream &os)
    {
        return print_interpreter_state(os);
    }

    int interpreter::cmd_mode(const nos::argv &argv, nos::ostream &os)
    {
        if (argv.size() >= 2)
        {
            std::string_view arg = argv[1];
            if (arg == "abs" || arg == "absolute" || arg == "90")
                _absolute_mode = true;
            else if (arg == "rel" || arg == "relative" || arg == "91")
                _absolute_mode = false;
            else
            {
                nos::println_to(os, "Usage: mode [abs|rel]");
                return 0;
            }
        }
        nos::fprintln_to(os, "mode: {} ({})",
                         _absolute_mode ? "absolute" : "relative",
                         _absolute_mode ? "G90" : "G91");
        return 0;
    }

    int interpreter::cmd_guard_info(const nos::argv &, nos::ostream &os)
    {
        return feedback_guard->guard_info(os);
    }

    int interpreter::cmd_planner_pause(const nos::argv &argv, nos::ostream &os)
    {
        if (argv.size() < 2)
        {
            nos::println_to(os, "Usage: planner_pause <0|1>");
            nos::println_to(os, "  1 = pause, 0 = resume");
            return 0;
        }
        auto en = std::stoi(argv[1]);
        planner->set_pause_mode(en);
        return 0;
    }

    int interpreter::cmd_set_junction_deviation(const nos::argv &argv, nos::ostream &os)
    {
        if (argv.size() < 2)
        {
            nos::fprintln_to(os, "junction_deviation: {} mm", _lookahead.junction_deviation());
            nos::fprintln_to(os, "junction_deviation_steps: {}", _lookahead.junction_deviation_steps());
            nos::fprintln_to(os, "lookahead_enabled: {}", _lookahead.is_enabled());
            nos::println_to(os, "Usage: set_junction_deviation <value_mm>");
            nos::println_to(os, "  0 = disable look-ahead");
            nos::println_to(os, "  Typical values: 0.01-0.05 mm");
            return 0;
        }
        cnc_float_type val = igris_atof64(argv[1].data(), NULL);
        set_junction_deviation(val);
        nos::fprintln_to(os, "junction_deviation set to {} mm", val);
        if (val > 0)
            nos::fprintln_to(os, "look-ahead enabled ({} steps)", _lookahead.junction_deviation_steps());
        else
            nos::println_to(os, "look-ahead disabled");
        return 0;
    }

    int interpreter::cmd_queue_status(const nos::argv &, nos::ostream &os)
    {
        nos::fprintln_to(os, "queue_capacity: {}", blocks->size());
        nos::fprintln_to(os, "queue_used: {}", blocks->avail());
        nos::fprintln_to(os, "queue_room: {}", blocks->room());
        nos::fprintln_to(os, "active_block: {}", planner->active_block != nullptr);
        nos::fprintln_to(os, "flow_control: {}", flow ? (flow->is_enabled() ? "enabled" : "disabled") : "not configured");
        return 0;
    }

    int interpreter::cmd_flow_control(const nos::argv &argv, nos::ostream &os)
    {
        if (!flow)
        {
            nos::println_to(os, "flow_control: not configured");
            return 0;
        }
        if (argv.size() < 2)
        {
            nos::fprintln_to(os, "flow_control: {}", flow->is_enabled() ? "enabled" : "disabled");
            return 0;
        }
        auto en = std::stoi(argv[1]);
        flow->set_enabled(en != 0);
        nos::fprintln_to(os, "flow_control: {}", flow->is_enabled() ? "enabled" : "disabled");
        return 0;
    }

    int interpreter::cmd_buffer(const nos::argv &argv, nos::ostream &os)
    {
        if (argv.size() < 2)
        {
            nos::println_to(os, "Usage: buffer <command>");
            nos::println_to(os, "  enable  - start buffering mode");
            nos::println_to(os, "  start   - execute buffered commands");
            nos::println_to(os, "  cancel  - cancel buffering and clear queue");
            nos::println_to(os, "  status  - show buffer status");
            nos::println_to(os, "  config <min_blocks> [timeout_ms] - set auto-buffer params");
            return 0;
        }

        std::string_view cmd = argv[1];

        if (cmd == "enable")
        {
            buffer_enable();
            nos::println_to(os, "buffer: enabled, waiting for commands");
            return 0;
        }
        else if (cmd == "start")
        {
            if (!_buffer.is_explicit_mode())
            {
                nos::println_to(os, "buffer: not in buffer mode");
                return 0;
            }
            int queued = blocks->avail();
            buffer_start();
            nos::fprintln_to(os, "buffer: started {} blocks", queued);
            return 0;
        }
        else if (cmd == "cancel")
        {
            buffer_cancel();
            nos::println_to(os, "buffer: cancelled");
            return 0;
        }
        else if (cmd == "status")
        {
            nos::fprintln_to(os, "explicit_mode: {}", _buffer.is_explicit_mode());
            nos::fprintln_to(os, "min_blocks_to_start: {}", _buffer.min_blocks_to_start());
            nos::fprintln_to(os, "buffer_timeout_ms: {}", _buffer.timeout_ms());
            nos::fprintln_to(os, "blocks_pending: {}", blocks->avail());
            nos::fprintln_to(os, "buffer_ready: {}", is_buffer_ready());
            nos::fprintln_to(os, "planner_paused: {}", planner->pause);
            return 0;
        }
        else if (cmd == "config")
        {
            if (argv.size() < 3)
            {
                nos::println_to(os, "Usage: buffer config <min_blocks> [timeout_ms]");
                nos::fprintln_to(os, "Current: min_blocks={}, timeout_ms={}",
                                 _buffer.min_blocks_to_start(), _buffer.timeout_ms());
                return 0;
            }
            int min_blocks = std::stoi(argv[2]);
            int timeout_ms = 100; // default 100ms
            if (argv.size() >= 4)
                timeout_ms = std::stoi(argv[3]);

            set_min_blocks_to_start(min_blocks);
            set_buffer_timeout_ms(timeout_ms);
            nos::fprintln_to(os, "buffer: min_blocks={}, timeout_ms={}",
                             min_blocks, timeout_ms);
            return 0;
        }
        else
        {
            nos::fprintln_to(os, "Unknown buffer command: {}", cmd);
            return 0;
        }
    }

    // === G-code methods ===

    void interpreter::g_command(const nos::argv &argv, nos::ostream &os)
    {
        int cmd = atoi(&argv[0].data()[1]);
        switch (cmd)
        {
        case 0: // G0 - rapid move
            command_rapid_move(argv.without(1), os);
            break;
        case 1: // G1 - linear move with feed
            command_linear_move(argv.without(1), os);
            break;
        case 90: // G90 - absolute mode
            _absolute_mode = true;
            break;
        case 91: // G91 - relative mode
            _absolute_mode = false;
            break;
        case 92: // G92 - set position
            command_G92(argv.without(1), os);
            break;
        default:
            nos::println_to(os, "Unresolved G command");
        }
    }

    void interpreter::m_command(const nos::argv &argv, nos::ostream &os)
    {
        int cmd = atoi(&argv[0].data()[1]);
        switch (cmd)
        {
        case 112:
            command_M204(argv.without(1), os);
            break;
        case 204:
            command_M204(argv.without(1), os);
            break;
        default:
            nos::println_to(os, "Unresolved M command");
        }
    }

    int interpreter::gcode(const nos::argv &argv, nos::ostream &os)
    {
        if (argv.size() == 0)
            return 0;

        char cmdsymb = argv[0][0];
        switch (cmdsymb)
        {
        case 'G':
        {
            g_command(argv, os);
            break;
        }

        case 'M':
        {
            m_command(argv, os);
            break;
        }

        default:
            nos::fprintln_to(os, "Unresolved command: {}", argv[0]);
        }

        return 0;
    }

    int interpreter::command_help(nos::ostream &os)
    {
        for (auto &rec : clicommands)
        {
            nos::fprintln_to(os, "{} - {}", rec.key, rec.help);
        }
        return 0;
    }

    int interpreter::gcode_help(nos::ostream &os)
    {
        nos::println_to(os, "G0 <axis><pos>... [M<accel>] - rapid move (max velocity)");
        nos::println_to(os, "G1 <axis><pos>... F<feed> [M<accel>] - linear move");
        nos::println_to(os, "  G0/G1 respect G90/G91 mode");
        nos::println_to(os, "  Example: G1 X10 Y-5 F100 M500");
        nos::println_to(os, "G90 - absolute positioning mode (default)");
        nos::println_to(os, "G91 - relative/incremental positioning mode");
        nos::println_to(os, "G92 <axis><pos>... - set position without moving");
        nos::println_to(os, "  Example: G92 X0 Y0 Z0");
        nos::println_to(os, "M112 - emergency stop");
        nos::println_to(os, "M204 [<accel>] - get/set default acceleration");
        return 0;
    }

    int interpreter::print_interpreter_state(nos::ostream &os)
    {
        PRINTTO(os, revolver_frequency);
        PRINTTO(os, saved_acc);
        PRINTTO(os, saved_feed);
        nos::print_to(os, "cur_vel: ");
        nos::print_list_to(os, revolver->current_velocities());
        nos::println_to(os);
        nos::print_to(os, "cur_acc:");
        nos::print_list_to(os, planner->accelerations);
        nos::println_to(os);
        nos::print_to(os, "velmaxs:");
        nos::print_list_to(os, max_axes_velocities);
        nos::println_to(os);
        nos::print_to(os, "accmaxs:");
        nos::print_list_to(os, max_axes_accelerations);
        nos::println_to(os);
        nos::print_to(os, "gains:");
        nos::print_list_to(os, gains);
        nos::println_to(os);
        nos::print_to(os, "gears:");
        nos::print_list_to(os, _steps_per_unit);
        nos::println_to(os);
        nos::print_to(os, "active_block: ");
        nos::println_to(os, planner->active_block != nullptr);
        nos::print_to(os, "active: ");
        nos::println_to(os, planner->active);
        nos::print_to(os, "head: ");
        nos::println_to(os, planner->blocks->head_index());
        nos::println_to(os);
        // Look-ahead status
        nos::fprintln_to(os, "lookahead_enabled: {}", _lookahead.is_enabled());
        nos::fprintln_to(os, "junction_deviation: {} mm ({} steps)",
                         _lookahead.junction_deviation(), _lookahead.junction_deviation_steps());
        // Positioning mode
        nos::fprintln_to(os, "positioning_mode: {} ({})",
                         _absolute_mode ? "absolute" : "relative",
                         _absolute_mode ? "G90" : "G91");
        return 0;
    }
}
