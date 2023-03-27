#include <ralgo/cnc/revolver.h>

cnc::revolver::revolver(igris::ring<cnc::control_shift> *ring)
    : shifts_ring(ring)
{
}

void cnc::revolver::cleanup()
{
    shifts_ring->reset();
}

bool cnc::revolver::is_empty()
{
    igris::syslock_guard guard;
    return shifts_ring->empty();
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

int cnc::revolver::queue_size()
{
    int size;

    system_lock();
    size = shifts_ring->avail();
    system_unlock();

    return size;
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
    int ret = shifts_ring->room();
    system_unlock();

    return ret;
}

void cnc::revolver::push(uint16_t step, uint16_t dir)
{
    // Добавление данных в очередь не требует блокировки,
    // потому что ring_head lockfree на добавление и чтение,
    // если unsigned int атомарен.

    shifts_ring->emplace(step, dir);
}

void cnc::revolver::serve()
{
    if (shifts_ring->empty())
        return;

    auto &shift = shifts_ring->tail();

    for (int i = 0; i < steppers_total; ++i)
    {
        revolver_t mask = 1 << i;
        bool step = shift.step & mask;

        if (!step)
            continue;

        bool dir = shift.direction & mask;
        auto &stepper = *steppers[i];

        if (step)
        {
            if (dir)
                stepper.inc();
            else
                stepper.dec();
        }
        else
        {
            stepper.no_action();
        }
    }

    shifts_ring->move_tail_one();
    return;
}

void cnc::revolver::clear()
{
    shifts_ring->clear();
}