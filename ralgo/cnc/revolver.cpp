#include <ralgo/cnc/revolver.h>

cnc::revolver::revolver() : revolver_task_ring(100)
{
    std::fill(dda_counters_fixed.begin(), dda_counters_fixed.end(), 0);
    std::fill(velocities_fixed.begin(), velocities_fixed.end(), 0);
    current_revolver_task = nullptr;
}

bool cnc::revolver::is_empty()
{
    igris::syslock_guard guard;
    return revolver_task_ring.empty();
}

robo::stepper **cnc::revolver::get_steppers()
{
    return steppers;
}

void cnc::revolver::set_steppers(robo::stepper **steppers_table, int size)
{
    steppers = steppers_table;
    steppers_total = size;
}

void cnc::revolver::current_steps(int64_t *steps)
{
    system_lock();
    for (int i = 0; i < steppers_total; ++i)
        steps[i] = steppers[i]->steps_count();
    system_unlock();
}

std::vector<int64_t> cnc::revolver::current_steps()
{
    std::vector<int64_t> vec(steppers_total);
    system_lock();
    for (int i = 0; i < steppers_total; ++i)
        vec[i] = steppers[i]->steps_count();
    system_unlock();
    return vec;
}

std::vector<int64_t> cnc::revolver::current_steps_no_lock()
{
    std::vector<int64_t> vec(steppers_total);
    for (int i = 0; i < steppers_total; ++i)
        vec[i] = steppers[i]->steps_count();
    return vec;
}

int cnc::revolver::room()
{
    // Операция взятия оставшегося места над кольцевым буфером
    // требует сравнения head и tail, изменяемых в разных потоках,
    // поэтому не является атомарной.
    system_lock();
    int ret = revolver_task_ring.room();
    system_unlock();

    return ret;
}

void cnc::revolver::cleanup()
{
    system_lock();
    std::fill(dda_counters_fixed.begin(), dda_counters_fixed.end(), 0);
    std::fill(velocities_fixed.begin(), velocities_fixed.end(), 0);
    revolver_task_ring.clear();
    current_revolver_task = nullptr;
    system_unlock();
}

void cnc::revolver::serve()
{
    if (current_revolver_task == nullptr && !revolver_task_ring.empty())
    {
        current_revolver_task = &revolver_task_ring.tail();
        counter = current_revolver_task->counter;
    }

    if (current_revolver_task == nullptr)
    {
        return;
    }

    for (int i = 0; i < steppers_total; ++i)
    {
        dda_counters_fixed[i] +=
            velocities_fixed[i] +
            (current_revolver_task->accelerations_fixed[i] >> 1);

        int64_t double_gears = gears_fixed[i] + gears_fixed[i];
        assert(dda_counters_fixed[i] >= -double_gears &&
               dda_counters_fixed[i] <= double_gears);

        auto &stepper = *steppers[i];
        if (dda_counters_fixed[i] > gears_high_trigger_fixed[i])
        {
            dda_counters_fixed[i] -= gears_fixed[i];
            stepper.inc();
        }
        else if (dda_counters_fixed[i] < -gears_high_trigger_fixed[i])
        {
            stepper.dec();
            dda_counters_fixed[i] += gears_fixed[i];
        }
        velocities_fixed[i] += current_revolver_task->accelerations_fixed[i];
    }

    counter--;

    if (counter == 0)
    {
        revolver_task_ring.move_tail_one();
        current_revolver_task = nullptr;
    }
}

void cnc::revolver::clear()
{
    system_lock();
    revolver_task_ring.clear();
    current_revolver_task = nullptr;
    counter = 0;
    std::fill(dda_counters_fixed.begin(), dda_counters_fixed.end(), 0);
    std::fill(velocities_fixed.begin(), velocities_fixed.end(), 0);
    system_unlock();
}

void cnc::revolver::clear_no_velocity_drop()
{
    system_lock();
    revolver_task_ring.clear();
    current_revolver_task = nullptr;
    counter = 0;
    system_unlock();
}