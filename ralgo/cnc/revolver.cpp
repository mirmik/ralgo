#include <ralgo/cnc/revolver.h>
#include <ralgo/cnc/util.h>

cnc::revolver::revolver() : task_queue(100)
{
    positions_fixed.fill(0);
    velocities_fixed.fill(0);
    current_task = nullptr;
}

void cnc::revolver::set_steppers(robo::stepper **steppers_table, int count)
{
    steppers = steppers_table;
    steppers_total = count;
}

robo::stepper **cnc::revolver::get_steppers()
{
    return steppers;
}

bool cnc::revolver::is_empty()
{
    igris::syslock_guard guard;
    return task_queue.empty() && current_task == nullptr;
}

int cnc::revolver::queue_room()
{
    igris::syslock_guard guard;
    return task_queue.room();
}

int cnc::revolver::queue_size()
{
    igris::syslock_guard guard;
    return task_queue.avail();
}

void cnc::revolver::advance_to_next_task()
{
    if (!task_queue.empty())
    {
        current_task = &task_queue.tail();
        ticks_remaining = current_task->duration_ticks;
    }
    else
    {
        current_task = nullptr;
        ticks_remaining = 0;
    }
}

void cnc::revolver::serve()
{
    // Get next task if needed
    if (current_task == nullptr)
    {
        if (task_queue.empty())
            return;
        advance_to_next_task();
    }

    if (current_task == nullptr)
        return;

    // DDA integration for each axis
    for (int i = 0; i < steppers_total; ++i)
    {
        // Integrate: velocity += acceleration (using midpoint for better accuracy)
        fixed_t half_accel = current_task->accelerations_fixed[i] >> 1;
        positions_fixed[i] += velocities_fixed[i] + half_accel;
        velocities_fixed[i] += current_task->accelerations_fixed[i];

        // Generate steps when position crosses integer boundaries
        auto &stepper = *steppers[i];

        while (positions_fixed[i] >= FIXED_POINT_MUL)
        {
            positions_fixed[i] -= FIXED_POINT_MUL;
            stepper.inc();
        }

        while (positions_fixed[i] < 0)
        {
            positions_fixed[i] += FIXED_POINT_MUL;
            stepper.dec();
        }
    }

    // Advance tick counter
    ticks_remaining--;

    if (ticks_remaining <= 0)
    {
        task_queue.move_tail_one();
        advance_to_next_task();
    }
}

void cnc::revolver::get_current_steps(steps_t *out_steps)
{
    igris::syslock_guard guard;
    for (int i = 0; i < steppers_total; ++i)
    {
        out_steps[i] = steppers[i]->steps_count();
    }
}

std::vector<steps_t> cnc::revolver::get_current_steps()
{
    std::vector<steps_t> result(steppers_total);
    igris::syslock_guard guard;
    for (int i = 0; i < steppers_total; ++i)
    {
        result[i] = steppers[i]->steps_count();
    }
    return result;
}

std::vector<steps_t> cnc::revolver::get_current_steps_no_lock()
{
    std::vector<steps_t> result(steppers_total);
    for (int i = 0; i < steppers_total; ++i)
    {
        result[i] = steppers[i]->steps_count();
    }
    return result;
}

std::array<cnc_float_type, NMAX_AXES> cnc::revolver::get_current_velocities()
    const
{
    std::array<cnc_float_type, NMAX_AXES> result = {};
    for (size_t i = 0; i < NMAX_AXES; ++i)
    {
        result[i] =
            static_cast<cnc_float_type>(velocities_fixed[i]) / FIXED_POINT_MUL;
    }
    return result;
}

void cnc::revolver::clear()
{
    igris::syslock_guard guard;
    task_queue.clear();
    current_task = nullptr;
    ticks_remaining = 0;

    // Final position correction: if fractional part > 0.5, round up
    // This eliminates the last-step error from integration truncation
    for (int i = 0; i < steppers_total; ++i)
    {
        if (positions_fixed[i] >= FIXED_POINT_MUL / 2)
        {
            steppers[i]->inc();
        }
        else if (positions_fixed[i] <= -FIXED_POINT_MUL / 2)
        {
            steppers[i]->dec();
        }
    }

    positions_fixed.fill(0);
    velocities_fixed.fill(0);
}

void cnc::revolver::clear_keep_velocity()
{
    igris::syslock_guard guard;
    task_queue.clear();
    current_task = nullptr;
    ticks_remaining = 0;
    // Keep velocities for smooth stop
}

void cnc::revolver::reset()
{
    clear();
}
