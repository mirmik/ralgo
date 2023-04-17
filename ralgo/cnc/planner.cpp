#include <ralgo/cnc/planner.h>

void cnc::planner::cleanup()
{
    blocks->clear();
    shifts->clear();
    shifts->reset();
    blocks->reset();
    active_block = nullptr;
    active = blocks->head_index();
    state = 0;
    std::fill(velocities.begin(), velocities.end(), 0);
    std::fill(dda_counters.begin(), dda_counters.end(), 0);
    std::fill(accelerations.begin(), accelerations.end(), 0);
    need_to_reevaluate = false;
    state = 0;
    waited = 0;
}

bool cnc::planner::is_dda_overflow_detected()
{
    return dda_counter_overflow_error_detected;
}

void cnc::planner::disable_frequency_protection()
{
    _frequency_protection = false;
}

void cnc::planner::set_start_operation_handle(igris::delegate<void> dlg)
{
    _start_operation_handle = dlg;
}

void cnc::planner::force_skip_all_blocks()
{
    blocks->clear();
    active_block = nullptr;
    active = blocks->head_index();
    state = 0;
    std::fill(accelerations.begin(), accelerations.end(), 0);
    // memset(velocities, 0, sizeof(velocities));
    // memset(dda_counters, 0, sizeof(dda_counters));
}

void cnc::planner::set_current_velocity(const std::vector<double> &vel)
{
    for (int i = 0; i < total_axes; ++i)
    {
        velocities[i] = vel[i];
    }
}

void cnc::planner::set_current_velocity(
    const std::array<double, NMAX_AXES> &vel)
{
    velocities = vel;
}

void cnc::planner::update_triggers()
{
    for (unsigned int i = 0; i < gears.size(); ++i)
    {
        gears_high_trigger[i] = gears[i] * 0.9;
    }
}

void cnc::planner::set_dim(int axes)
{
    total_axes = axes;
}

void cnc::planner::reset_iteration_counter()
{
    iteration_counter = 0;
}

cnc::planner::planner(igris::ring<cnc::planner_block> *blocks,
                      igris::ring<cnc::control_shift> *shifts)
    : blocks(blocks), shifts(shifts)
{
    std::fill(accelerations.begin(), accelerations.end(), 0);
    std::fill(dda_counters.begin(), dda_counters.end(), 0);
}

int cnc::planner::block_index(planner_block *it)
{
    return blocks->index_of(it);
}

void cnc::planner::fixup_postactive_blocks()
{
    while (blocks->tail_index() != active)
    {
        if (!blocks->tail().is_active_or_postactive(iteration_counter))
            blocks->pop();

        else
            break;
    }
}

bool cnc::planner::has_postactive_blocks()
{
    system_lock();
    auto ret = active != blocks->tail_index();
    system_unlock();
    return ret;
}

int cnc::planner::count_of_postactive()
{
    system_lock();
    auto ret = blocks->distance(active, blocks->tail_index());
    system_unlock();
    return ret;
}

void cnc::planner::change_active_block()
{
    if (info_mode)
    {
        ralgo::info("planner: change_active_block");
    }

    system_lock();
    if (active_block && has_postactive_blocks() == 0 &&
        iteration_counter == active_block->active_finish_ic)
    {
        // TODO: normalize time
    }

    if (active_block)
        active = blocks->fixup_index(active + 1);

    int head = blocks->head_index();
    system_unlock();

    if (active_block == nullptr && active != head)
    {
        _start_operation_handle();
    }

    if (active == head)
    {
        active_block = nullptr;
        return;
    }

    active_block = &blocks->get(active);

    assert(active_block->blockno == waited);
    waited++;

    active_block->shift_timestampes(iteration_counter);
}

const std::array<double, NMAX_AXES> &cnc::planner::current_velocity()
{
    return velocities;
}

int cnc::planner::serve()
{
    int final;

    system_lock();
    bool shifts_empty = shifts->empty();
    bool blocks_empty = blocks->empty();
    if (shifts_empty && blocks_empty && active_block == nullptr && in_operation)
    {
        in_operation = false;
        final_shift_pushed();
        system_unlock();
        return 0;
    }
    int room = shifts->room();
    system_unlock();

    if (blocks_empty && active_block == nullptr)
        return 0;

    while (room--)
    {
        // Планируем поведение револьвера на несколько циклов вперёд
        // попутно инкрементируя модельное время.
        final = iteration();
        in_operation = true;

        if (final)
            return final;
    }

    return 0;
}

void cnc::planner::evaluate_accelerations()
{
    if (active_block)
        active_block->assign_accelerations(
            accelerations.data(), total_axes, iteration_counter);
    else
    {
        for (int i = 0; i < total_axes; ++i)
            accelerations[i] = 0;
    }

    for (int i = blocks->tail_index(); i != active;
         i = blocks->fixup_index(i + 1))
        blocks->get(i).append_accelerations(
            accelerations.data(), total_axes, iteration_counter);
}

/// В этой фазе расчитывается программе револьвера
/// на основе интегрирования ускорений и скоростей.
void cnc::planner::iteration_planning_phase()
{
    revolver_t mask, step = 0, dir = 0;

    for (int i = 0; i < total_axes; ++i)
    {
        mask = (1 << i);

        dda_counters[i] += velocities[i] +         //* delta +
                           accelerations[i] * 0.5; //* delta_sqr_div_2;

        // check frequency correctness
        if (_frequency_protection)
        {
            int gears_per_counter = dda_counters[i] / gears[i];
            assert(gears_per_counter >= -1 && gears_per_counter <= 1);
        }

        if (dda_counters[i] > gears_high_trigger[i])
        {
            dda_counters[i] -= gears[i];
            dir |= mask;
            step |= mask;

            if (dda_counters[i] >= gears[i])
            {
                dda_counter_overflow_error_detected = true;
            }
        }
        else if (dda_counters[i] < -gears_high_trigger[i])
        {
            dda_counters[i] += gears[i];
            dir |= mask;

            if (dda_counters[i] <= gears[i])
            {
                dda_counter_overflow_error_detected = true;
            }
        }
        velocities[i] += accelerations[i]; // * delta;
    }
    system_lock();
    shifts->emplace(dir, step);
    system_unlock();

    iteration_counter++;
}

int cnc::planner::iteration()
{
    if (active_block)
    {
        if (state == 0)
        {
            if (!active_block->is_accel(iteration_counter))
            {
                need_to_reevaluate = true;
                state = 1;
            }
        }

        else
        {
            if (!active_block->is_active(iteration_counter))
            {
                change_active_block();
                need_to_reevaluate = true;
                state = 0;
            }
        }
    }

    for (int i = blocks->tail_index(); i != active;
         i = blocks->fixup_index(i + 1))
    {
        if (!blocks->get(i).is_active_or_postactive(iteration_counter))
        {
            need_to_reevaluate = true;
        }
    }

    if (active_block == nullptr && !has_postactive_blocks())
    {
        bool empty = blocks->empty();
        if (empty)
            return 1;

        change_active_block();

        if (active_block == nullptr)
            return 1;

        need_to_reevaluate = true;
    }

    if (need_to_reevaluate)
    {
        fixup_postactive_blocks();
        evaluate_accelerations();
        need_to_reevaluate = false;
        // count_of_reevaluation++;
    }

    if (active_block == nullptr && !has_postactive_blocks())
        return 1;

    iteration_planning_phase();
    return 0;
}

void cnc::planner::set_axes_count(int total)
{
    total_axes = total;
    gears.resize(total);
    gears_high_trigger.resize(total);
    ralgo::vecops::fill(gears, 1000);
    update_triggers();
}

void cnc::planner::set_gears(const igris::array_view<double> &arr)
{
    ralgo::vecops::copy(arr, gears);
    update_triggers();
}

igris::array_view<double> cnc::planner::get_gears()
{
    return {gears.data(), gears.size()};
}

size_t cnc::planner::get_total_axes()
{
    return total_axes;
}

void cnc::planner::set_gear(int index, double val)
{
    system_lock();
    gears[index] = val;
    update_triggers();
    system_unlock();
}

void cnc::planner::clear()
{
    blocks->clear();
    std::fill(dda_counters.begin(), dda_counters.end(), 0);
    std::fill(accelerations.begin(), accelerations.end(), 0);
    std::fill(velocities.begin(), velocities.end(), 0);
    active_block = nullptr;
    active = blocks->head_index();
    need_to_reevaluate = true;
    state = 0;
    change_active_block();
}

void cnc::planner::clear_queue()
{
    blocks->set_last_index(active);
}
