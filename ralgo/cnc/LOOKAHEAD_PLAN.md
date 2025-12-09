# План реализации Look-ahead Planning

## Проблема

Сейчас в `planner_block::set_state()` (строка 270-271):
```cpp
start_velocity = 0;
final_velocity = 0;
```

Каждый блок начинается и заканчивается с нулевой скоростью — машина полностью останавливается между сегментами:

```
Сейчас (start=0, final=0) - полная остановка между блоками:

Блок 1       Блок 2       Блок 3
   /\           /\           /\
  /  \         /  \         /  \
 /    \       /    \       /    \
/      \_____/      \_____/      \
       stop        stop
```

## Цель

Плавное сопряжение блоков на стыках:

```
С look-ahead - плавное сопряжение на стыках:

Блок 1       Блок 2       Блок 3
   /‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\
  /    \     /\     /    \     \
 /      \   /  \   /      \     \
/        \_/    \_/        \     \
      junction  junction
      velocity  velocity
```

На стыках скорость не падает до нуля, а снижается до **junction velocity** — значения, зависящего от угла между сегментами.

## Алгоритм Junction Deviation

Используется в Marlin/Grbl. Основная идея: ограничить центростремительное ускорение на повороте.

```cpp
// Угол между блоками
cos_theta = dot(dir1, dir2)

// Junction velocity на основе центростремительного ускорения
sin_half = sqrt(0.5 * (1 - cos_theta))
v_junction = sqrt(junction_deviation * acceleration / sin_half)
```

Поведение:
- `cos_theta = 1` (прямая линия) → `sin_half → 0` → `v_junction → ∞` (ограничиваем nominal)
- `cos_theta = 0` (угол 90°) → `sin_half = 0.707` → заметное снижение скорости
- `cos_theta = -1` (разворот 180°) → `sin_half = 1` → минимальная скорость

Параметр `junction_deviation` (типичное значение 0.01-0.05 мм) определяет допустимое отклонение от идеальной траектории на повороте.

## План реализации

### Шаг 1: Расчёт Junction Velocity (planblock.h)

Добавить статическую функцию:

```cpp
/// Рассчитать максимальную скорость на стыке двух блоков
/// @param dir1 - направление первого блока (нормализованное)
/// @param dir2 - направление второго блока (нормализованное)
/// @param acceleration - ускорение (steps/tick²)
/// @param junction_deviation - допустимое отклонение (steps)
/// @return максимальная скорость на стыке (steps/tick)
static cnc_float_type calculate_junction_velocity(
    const std::array<cnc_float_type, NMAX_AXES>& dir1,
    const std::array<cnc_float_type, NMAX_AXES>& dir2,
    cnc_float_type acceleration,
    cnc_float_type junction_deviation,
    int axes);
```

### Шаг 2: Модификация set_state() (planblock.h)

Изменить сигнатуру для приёма граничных скоростей:

```cpp
void set_state(const ralgo::vector_view<cnc_float_type> &axdist,
               int axes,
               cnc_float_type velocity,
               cnc_float_type acceleration,
               cnc_float_type start_vel = 0,    // NEW
               cnc_float_type final_vel = 0);   // NEW
```

Пересчёт таймингов с учётом ненулевых граничных скоростей:

```
Трапеция с start_velocity > 0 и final_velocity > 0:

        B______________C
       /                \
      /                  \
     /                    \
    A                      D
    ^                      ^
start_velocity      final_velocity
```

Формулы:
- Время разгона: `t_accel = (nominal - start_velocity) / acceleration`
- Время торможения: `t_decel = (nominal - final_velocity) / acceleration`
- Расстояние разгона: `d_accel = (start² + nominal²) / 2 * t_accel`
- И т.д.

### Шаг 3: Функция пересчёта профиля (planblock.h)

```cpp
/// Пересчитать тайминги блока с новыми граничными скоростями
/// Вызывается при recalculate после backward/forward pass
void recalculate_timing(cnc_float_type new_start_vel,
                        cnc_float_type new_final_vel);
```

### Шаг 4: Backward/Forward Pass (planner.h/cpp)

Добавить в planner:

```cpp
/// Пересчитать скорости всех блоков в очереди
/// Вызывается при добавлении нового блока
void recalculate_block_velocities();

private:
    /// Backward pass: гарантировать возможность торможения
    void backward_pass();

    /// Forward pass: гарантировать возможность разгона
    void forward_pass();
```

Алгоритм backward pass:
```cpp
void backward_pass() {
    // Идём от конца к началу
    for (int i = last_block; i > first_block; --i) {
        auto& curr = blocks[i];
        auto& prev = blocks[i-1];

        // Максимальная скорость входа в curr при торможении с final_velocity
        cnc_float_type max_entry = sqrt(
            curr.final_velocity² + 2 * curr.acceleration * curr.fullpath
        );

        // Ограничить выходную скорость предыдущего блока
        prev.final_velocity = min(prev.final_velocity, max_entry);
        curr.start_velocity = prev.final_velocity;
    }
}
```

Алгоритм forward pass:
```cpp
void forward_pass() {
    // Идём от начала к концу
    for (int i = first_block; i < last_block; ++i) {
        auto& curr = blocks[i];
        auto& next = blocks[i+1];

        // Максимальная скорость выхода из curr при разгоне с start_velocity
        cnc_float_type max_exit = sqrt(
            curr.start_velocity² + 2 * curr.acceleration * curr.fullpath
        );

        // Ограничить входную скорость следующего блока
        curr.final_velocity = min(curr.final_velocity, max_exit);
        next.start_velocity = curr.final_velocity;
    }
}
```

### Шаг 5: Интеграция в Interpreter (interpreter.h)

Добавить параметр и логику:

```cpp
class interpreter {
    // ...
    cnc_float_type _junction_deviation = 0.01;  // мм, конвертируется в steps

public:
    void set_junction_deviation(cnc_float_type mm);

private:
    /// Вычислить junction velocity между lastblock и новым блоком
    cnc_float_type compute_junction_velocity(const planner_block& prev,
                                              const planner_block& next);
};
```

В `evaluate_task()`:
```cpp
void evaluate_task(const control_task &task, nos::ostream &os) {
    // ... существующий код ...

    // Вычислить junction velocity с предыдущим блоком
    if (blockno > 0) {
        cnc_float_type jv = compute_junction_velocity(lastblock, newblock);
        lastblock.final_velocity = jv;
        newblock.start_velocity = jv;
    }

    // Добавить блок в очередь
    // ...

    // Запустить пересчёт скоростей
    planner->recalculate_block_velocities();
}
```

### Шаг 6: CLI команды

```cpp
{"set_junction_deviation",
 "Set junction deviation in mm. Args: <value>. Example: set_junction_deviation 0.02",
 [this](const nos::argv &argv, nos::ostream &os) {
     if (argv.size() < 2) {
         nos::println_to(os, "Current: ", _junction_deviation);
         return 0;
     }
     _junction_deviation = igris_atof64(argv[1].data(), NULL);
     return 0;
 }},
```

### Шаг 7: Тесты

```cpp
TEST_CASE("Look-ahead: junction velocity") {
    SUBCASE("Прямая линия - скорость сохраняется") {
        // Два блока в одном направлении
        // junction_velocity должна быть == nominal
    }

    SUBCASE("Угол 90° - скорость снижается") {
        // Блок X+, затем Y+
        // junction_velocity < nominal
    }

    SUBCASE("Разворот 180° - минимальная скорость") {
        // Блок X+, затем X-
        // junction_velocity близка к 0
    }

    SUBCASE("Backward pass - торможение возможно") {
        // Очередь блоков с уменьшающейся длиной
        // Проверить что final_velocity корректно ограничены
    }
}
```

## Порядок реализации

1. **planblock.h**: `calculate_junction_velocity()` - статическая функция
2. **planblock.h**: модификация `set_state()` для приёма start/final velocity
3. **planblock.h**: `recalculate_timing()` - пересчёт таймингов
4. **planner.h/cpp**: `recalculate_block_velocities()`, `backward_pass()`, `forward_pass()`
5. **interpreter.h**: интеграция junction_deviation, вызов пересчёта
6. **tests**: тесты для всех случаев
7. **CLI**: команда set_junction_deviation

## Важные моменты

1. **Единицы измерения**: junction_deviation задаётся в мм, но внутри всё в steps. Конвертация происходит в interpreter.

2. **Когда пересчитывать**: при каждом добавлении блока в очередь. Это O(n) где n - размер очереди, но очередь обычно маленькая (8-16 блоков).

3. **Блоки уже в исполнении**: не трогаем active_block и postactive блоки - пересчитываем только pending.

4. **exact_stop флаг**: если блок помечен exact_stop=1, его final_velocity должна быть 0 независимо от junction calculation.

5. **Первый и последний блок**: первый блок всегда start_velocity=0 (если машина стоит), последний - final_velocity=0 (если нет следующего блока или exact_stop).
