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
    // Precondition: revolver must not be null
    std::fill(accelerations.begin(), accelerations.end(), 0);
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

    // Block sequence is managed internally - blockno should match waited
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
}

size_t cnc::planner::get_total_axes()
{
    return _total_axes;
}

size_t cnc::planner::total_axes()
{
    return _total_axes;
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

void cnc::planner::recalculate_block_velocities(cnc_float_type junction_deviation)
{
    // Пересчитываем только pending блоки (от active+1 до head)
    // Active блок уже в исполнении - его не трогаем

    int head = blocks->head_index();
    int start_idx = active_block ? blocks->fixup_index(active + 1) : active;

    // Если нет pending блоков - выходим
    if (start_idx == head)
        return;

    // Шаг 1: Вычислить junction velocity между соседними блоками
    // и установить начальные значения start/final velocity

    int prev_idx = start_idx;
    for (int i = blocks->fixup_index(start_idx + 1); i != head;
         i = blocks->fixup_index(i + 1))
    {
        auto &prev = blocks->get(prev_idx);
        auto &curr = blocks->get(i);

        // Вычисляем junction velocity на основе угла между блоками
        cnc_float_type jv = planner_block::calculate_junction_velocity(
            prev.direction(), curr.direction(),
            std::min(prev.nominal_velocity, curr.nominal_velocity),
            std::min(prev.acceleration, curr.acceleration), junction_deviation,
            _total_axes);

        // Ограничиваем junction velocity номинальными скоростями блоков
        if (jv > prev.nominal_velocity)
            jv = prev.nominal_velocity;
        if (jv > curr.nominal_velocity)
            jv = curr.nominal_velocity;

        prev.final_velocity = jv;
        curr.start_velocity = jv;

        prev_idx = i;
    }

    // Последний блок должен остановиться (final_velocity = 0)
    // если нет следующего блока
    int last_idx = blocks->fixup_index(head - 1);
    if (last_idx != head)
    {
        blocks->get(last_idx).final_velocity = 0;
    }

    // Первый pending блок: если есть active, то start_velocity = junction с ним
    // если нет active - start_velocity = 0
    if (start_idx != head)
    {
        if (active_block)
        {
            auto &first_pending = blocks->get(start_idx);
            cnc_float_type jv = planner_block::calculate_junction_velocity(
                active_block->direction(), first_pending.direction(),
                std::min(active_block->nominal_velocity,
                         first_pending.nominal_velocity),
                std::min(active_block->acceleration, first_pending.acceleration),
                junction_deviation, _total_axes);

            // Ограничиваем
            if (jv > active_block->nominal_velocity)
                jv = active_block->nominal_velocity;
            if (jv > first_pending.nominal_velocity)
                jv = first_pending.nominal_velocity;

            first_pending.start_velocity = jv;
            // Примечание: active_block.final_velocity не меняем - он уже в исполнении
        }
        else
        {
            blocks->get(start_idx).start_velocity = 0;
        }
    }

    // Шаг 2: Backward pass - гарантируем возможность торможения
    backward_pass();

    // Шаг 3: Forward pass - гарантируем возможность разгона
    forward_pass();

    // Шаг 4: Пересчитываем тайминги всех pending блоков
    for (int i = start_idx; i != head; i = blocks->fixup_index(i + 1))
    {
        blocks->get(i).recalculate_timing();
    }
}

void cnc::planner::backward_pass()
{
    // Идём от конца к началу pending блоков
    // Для каждого блока проверяем: может ли он затормозить до final_velocity?
    // Если нет - снижаем final_velocity предыдущего блока

    int head = blocks->head_index();
    int start_idx = active_block ? blocks->fixup_index(active + 1) : active;

    if (start_idx == head)
        return;

    // Начинаем с предпоследнего блока (последний уже имеет final=0)
    int last_idx = blocks->fixup_index(head - 1);

    for (int i = last_idx; i != start_idx;)
    {
        int prev_idx = blocks->fixup_index(i - 1);
        if (prev_idx == blocks->fixup_index(start_idx - 1))
            break;

        auto &curr = blocks->get(i);
        auto &prev = blocks->get(prev_idx);

        // Максимальная скорость входа в curr при торможении до final_velocity
        // v_max² = v_final² + 2 * a * d
        cnc_float_type max_entry_sq =
            curr.final_velocity * curr.final_velocity +
            2 * curr.acceleration * curr.fullpath;
        cnc_float_type max_entry = sqrt(max_entry_sq);

        // Если start_velocity больше max_entry - снижаем
        if (curr.start_velocity > max_entry)
        {
            curr.start_velocity = max_entry;
            prev.final_velocity = max_entry;
        }

        i = prev_idx;
    }

    // Проверяем первый pending блок
    if (start_idx != head)
    {
        auto &first = blocks->get(start_idx);
        cnc_float_type max_entry_sq = first.final_velocity * first.final_velocity +
                                      2 * first.acceleration * first.fullpath;
        cnc_float_type max_entry = sqrt(max_entry_sq);

        if (first.start_velocity > max_entry)
        {
            first.start_velocity = max_entry;
        }
    }
}

void cnc::planner::forward_pass()
{
    // Идём от начала к концу pending блоков
    // Для каждого блока проверяем: может ли он разогнаться от start_velocity?
    // Если нет - снижаем final_velocity текущего блока и start следующего

    int head = blocks->head_index();
    int start_idx = active_block ? blocks->fixup_index(active + 1) : active;

    if (start_idx == head)
        return;

    for (int i = start_idx; i != head;)
    {
        int next_idx = blocks->fixup_index(i + 1);

        auto &curr = blocks->get(i);

        // Максимальная скорость выхода из curr при разгоне от start_velocity
        // v_max² = v_start² + 2 * a * d
        cnc_float_type max_exit_sq = curr.start_velocity * curr.start_velocity +
                                     2 * curr.acceleration * curr.fullpath;
        cnc_float_type max_exit = sqrt(max_exit_sq);

        // Если final_velocity больше max_exit - снижаем
        if (curr.final_velocity > max_exit)
        {
            curr.final_velocity = max_exit;
        }

        // Также ограничиваем nominal_velocity если не можем достичь
        if (curr.nominal_velocity > max_exit)
        {
            // Пиковая скорость будет меньше nominal
            // recalculate_timing() обработает это
        }

        // Обновляем start_velocity следующего блока
        if (next_idx != head)
        {
            auto &next = blocks->get(next_idx);
            if (next.start_velocity > curr.final_velocity)
            {
                next.start_velocity = curr.final_velocity;
            }
        }

        i = next_idx;
    }
}

#pragma GCC reset_options