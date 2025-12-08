#include <ralgo/cnc/planner.h>
#include <ralgo/cnc/util.h>

#pragma GCC optimize("Ofast")
void cnc::planner::cleanup()
{
    blocks->clear();
    blocks->reset();
    active_block = nullptr;
    active = blocks->head_index();
    state = 0;
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
}

void cnc::planner::update_triggers()
{
    // No longer needed - gears are stored in planner now
}

void cnc::planner::set_dim(int axes)
{
    _total_axes = axes;
}

void cnc::planner::reset_iteration_counter()
{
    iteration_counter = 0;
}

cnc::planner::planner(igris::ring<cnc::planner_block> *blocks,
                      cnc::revolver *revolver)
    : blocks(blocks), revolver(revolver)
{
    std::fill(accelerations.begin(), accelerations.end(), 0);
    assert(revolver);
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

void cnc::planner::evaluate_accelerations()
{
    if (active_block)
        active_block->assign_accelerations(
            accelerations.data(), _total_axes, iteration_counter);
    else
    {
        for (int i = 0; i < _total_axes; ++i)
            accelerations[i] = 0;
    }

    for (int i = blocks->tail_index(); i != active;
         i = blocks->fixup_index(i + 1))
        blocks->get(i).append_accelerations(
            accelerations.data(), _total_axes, iteration_counter);
}

void cnc::planner::set_pause_mode(bool en)
{
    pause = en;
}

int cnc::planner::serve(bool prevent_small_rooms)
{
    (void)prevent_small_rooms;

    if (pause)
        return 0;

    system_lock();
    bool revolver_empty = revolver->is_empty();
    bool blocks_empty = blocks->empty();
    if (revolver_empty && blocks_empty && active_block == nullptr &&
        in_operation)
    {
        in_operation = false;
        revolver->cleanup();
        final_shift_pushed();
        system_unlock();
        return 0;
    }
    system_unlock();

    system_lock();
    int room = revolver->room();
    system_unlock();

    if (blocks_empty && active_block == nullptr)
        return 0;

    while (room)
    {
        // Планируем поведение револьвера на несколько циклов вперёд
        // попутно инкрементируя модельное время.
        auto [fin, iterations] = iteration();
        in_operation = true;

        room -= 1;

        if (fin)
            return fin;
    }

    return 0;
}

/// В этой фазе расчитывается программе револьвера
/// на основе интегрирования ускорений и скоростей.
/// Ускорения уже в steps/tick² (конвертация mm→steps в interpreter)
void cnc::planner::iteration_planning_phase(size_t iter)
{
    if (revolver)
    {
        // accelerations already in steps/tick² (converted by interpreter)
        revolver->task_queue.emplace(accelerations, iter);
    }

    iteration_counter += iter;
}

std::pair<int, size_t> cnc::planner::iteration()
{
    size_t room = 10000;

    if (active_block == nullptr && !has_postactive_blocks())
    {
        bool empty = blocks->empty();
        if (empty)
        {
            return {1, 0};
        }

        // change_active_block может установить active_block,
        // поэтому значение проверяется повторно
        change_active_block();
        if (active_block == nullptr)
            return {1, 0};

        room = 1;
        need_to_reevaluate = true;
    }

    if (active_block)
    {
        // // room = 1;
        if (active_block->is_accel(iteration_counter))
        {
            int counter_to_active =
                active_block->acceleration_before_ic - iteration_counter;
            room = std::min(room, (size_t)counter_to_active);
            need_to_reevaluate = true;
        }
        else if (active_block->is_flat(iteration_counter))
        {
            int counter_to_active =
                active_block->deceleration_after_ic - iteration_counter;
            room = std::min(room, (size_t)counter_to_active);
            need_to_reevaluate = true;
        }
        else if (active_block->is_decel(iteration_counter))
        {
            int counter_to_active =
                active_block->block_finish_ic - iteration_counter;
            room = std::min(room, (size_t)counter_to_active);
            need_to_reevaluate = true;
        }

        if (room == 0)
            room = 1;

        if (state == 0)
        {
            if (!active_block->is_accel(iteration_counter))
            {
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
            room = 1;
        }
        else
        {
            int counter_to_finish =
                blocks->get(i).block_finish_ic - iteration_counter;
            room = std::min(room, (size_t)counter_to_finish);
        }
    }

    if (need_to_reevaluate)
        reevaluate_accelerations();

    iteration_planning_phase(room);

    return {0, room};
}

void cnc::planner::reevaluate_accelerations()
{
    fixup_postactive_blocks();
    evaluate_accelerations();
    need_to_reevaluate = false;
}

void cnc::planner::set_axes_count(int total)
{
    _total_axes = total;
    // Default steps_per_mm
    for (int i = 0; i < total; ++i)
    {
        _steps_per_mm[i] = 1000.0;
    }
}

void cnc::planner::set_gears(const igris::array_view<cnc_float_type> &arr)
{
    for (size_t i = 0; i < arr.size() && i < NMAX_AXES; ++i)
    {
        _steps_per_mm[i] = arr[i];
    }
}

std::array<cnc_float_type, NMAX_AXES> cnc::planner::get_gears()
{
    return _steps_per_mm;
}

size_t cnc::planner::get_total_axes()
{
    return _total_axes;
}

size_t cnc::planner::total_axes()
{
    return _total_axes;
}

void cnc::planner::set_gear(int index, cnc_float_type val)
{
    system_lock();
    _steps_per_mm[index] = val;
    system_unlock();
}

void cnc::planner::clear()
{
    blocks->clear();
    std::fill(accelerations.begin(), accelerations.end(), 0);
    active_block = nullptr;
    active = blocks->head_index();
    need_to_reevaluate = true;
    state = 0;
    waited = 0;
    revolver->clear();
    change_active_block();
}

void cnc::planner::clear_for_stop()
{
    blocks->set_last_index(active);
    blocks->clear();
    std::fill(accelerations.begin(), accelerations.end(), 0);
    active_block = nullptr;
    active = blocks->head_index();
    need_to_reevaluate = true;
    state = 0;
    waited = 0;
    revolver->clear_no_velocity_drop();
    change_active_block();
}

void cnc::planner::alarm_stop()
{
    system_lock();
    blocks->clear();
    std::fill(accelerations.begin(), accelerations.end(), 0);
    active_block = nullptr;
    active = blocks->head_index();
    need_to_reevaluate = true;
    state = 0;
    waited = 0;
    change_active_block();
    revolver->clear();
    system_unlock();
}

bool cnc::planner::is_not_halt()
{
    igris::syslock_guard lock;
    return !revolver->is_empty() || active_block != nullptr ||
           has_postactive_blocks() || !blocks->empty();
}

bool cnc::planner::is_halt()
{
    return !is_not_halt();
}

void cnc::planner::set_tick_frequency(uint32_t hz)
{
    _tick_frequency = hz;
}

uint32_t cnc::planner::get_tick_frequency() const
{
    return _tick_frequency;
}
#pragma GCC reset_options