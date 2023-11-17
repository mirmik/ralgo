#include <ralgo/cnc/revolver.h>

cnc::revolver::revolver() : revolver_task_ring(100)
{
    ralgo::vecops::fill(dda_counters, 0);
    ralgo::vecops::fill(velocities, 0);
}

void cnc::revolver::cleanup()
{
    // shifts_ring->reset();
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

// int cnc::revolver::queue_size()
// {
//     int size;

//     system_lock();
//     size = shifts_ring->avail();
//     system_unlock();

//     return size;
// }

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

// void cnc::revolver::push(uint16_t step, uint16_t dir)
// {
//     // Добавление данных в очередь не требует блокировки,
//     // потому что ring_head lockfree на добавление и чтение,
//     // если unsigned int атомарен.

//     // shifts_ring->emplace(step, dir);
// }

void cnc::revolver::serve()
{
    // if (shifts_ring->empty())
    //     return;

    if (current_revolver_task == nullptr && !revolver_task_ring.empty())
    {
        current_revolver_task = &revolver_task_ring.tail();
        iteration_counter = current_revolver_task->step_start;

        // nos::println("REVOLVER TASK START");
        // nos::println("step_start", current_revolver_task->step_start);
        // nos::println("step_end", current_revolver_task->step_end);
        // nos::println("accelerations", current_revolver_task->accelerations);
        // std::cout.flush();
    }

    if (current_revolver_task == nullptr)
        return;

    // if (iteration_counter >= 3)
    // {
    //     exit(0);
    // }

    for (int i = 0; i < steppers_total; ++i)
    {
        // mask = (1 << i);

        // if (current_revolver_task->accelerations[i] < 0)
        // {
        //     nos::println(current_revolver_task->accelerations[i]);
        // }
        dda_counters[i] +=
            velocities[i] +                                //* delta +
            current_revolver_task->accelerations[i] * 0.5; //* delta_sqr_div_2;

        // nos::println("dda_counters[i]", dda_counters[i]);

        // check frequency correctness
        // int gears_per_counter = dda_counters[i] / gears[i];
        // assert(gears_per_counter >= -1 && gears_per_counter <= 1);

        auto &stepper = *steppers[i];
        if (dda_counters[i] > gears_high_trigger[i])
        {
            dda_counters[i] -= gears[i];
            stepper.inc();
            //    dir |= mask;
            //    step |= mask;
            // if (dda_counters[i] >= gears[i])
            // {
            //     dda_counter_overflow_error_detected = true;
            // }
        }
        else if (dda_counters[i] < -gears_high_trigger[i])
        {
            stepper.dec();
            //    dda_counters[i] += gears[i];
            //    dir |= mask;
            // if (dda_counters[i] <= gears[i])
            // {
            //     dda_counter_overflow_error_detected = true;
            // }
        }
        velocities[i] += current_revolver_task->accelerations[i]; // * delta;
    }

    // auto &shift = shifts_ring->tail();
    //(void)shift;

    // for (int i = 0; i < steppers_total; ++i)
    // {
    //     revolver_t mask = 1 << i;
    //     bool step = shift.step & mask;

    //     if (!step)
    //         continue;

    //     bool dir = shift.direction & mask;
    //     auto &stepper = *steppers[i];

    //     if (step)
    //     {
    //         if (dir)
    //             stepper.inc();
    //         else
    //             stepper.dec();
    //     }
    //     else
    //     {
    //         stepper.no_action();
    //     }
    // }

    // shifts_ring->move_tail_one();
    iteration_counter++;

    if (current_revolver_task &&
        iteration_counter >= current_revolver_task->step_end)
    {
        revolver_task_ring.move_tail_one();
        current_revolver_task = nullptr;
    }

    return;
}

void cnc::revolver::clear()
{
    nos::println("CLEAR");
    system_lock();
    // shifts_ring->clear();
    revolver_task_ring.clear();
    current_revolver_task = nullptr;
    iteration_counter = 0;
    std::fill(dda_counters.begin(), dda_counters.end(), 0);
    std::fill(velocities.begin(), velocities.end(), 0);
    system_unlock();
}

void cnc::revolver::clear_no_velocity_drop()
{
    nos::println("CLEAR_NO_VELOCITY_DROP");
    system_lock();
    // shifts_ring->clear();
    revolver_task_ring.clear();
    current_revolver_task = nullptr;
    iteration_counter = 0;
    // std::fill(dda_counters.begin(), dda_counters.end(), 0);
    // std::fill(velocities.begin(), velocities.end(), 0);
    system_unlock();
}