# План улучшения CNC-контроллера

## Оценка текущего состояния

### Обзор архитектуры

CNC-контроллер имеет чистую многоуровневую архитектуру:

```
G-code (mm, mm/sec)
    |
    v
Interpreter --> преобразование единиц (mm -> steps)
    |
    v
Planner --> очередь блоков движения, трапецеидальные профили
    |
    v
Revolver --> DDA интегрирование в fixed-point (2^24)
    |
    v
Hardware (драйверы шаговых двигателей)
```

**Ключевой принцип:** Single Point of Conversion — все преобразования единиц происходят только в Interpreter. Planner и Revolver работают с абстрактными steps/tick.

### Состав кода (~3200 строк)

| Компонент | Строк | Назначение |
|-----------|-------|------------|
| `interpreter.h` | 1065 | Главный оркестратор, парсинг G-code, конверсия единиц |
| `planner.h/.cpp` | 485 | Менеджер очереди блоков, трапецеидальные профили |
| `revolver.h/.cpp` | 294 | DDA генератор импульсов в fixed-point |
| `planblock.h` | 360 | Структура блока с профилем скорости |
| `feedback_guard.h` | 383 | Контроль обратной связи, tandem-оси |
| `error_handler.h` | 245 | Lock-free обработка ошибок в реальном времени |

### Реализованный функционал

- G1 линейная интерполяция (до 10 осей)
- Трапецеидальный профиль скорости (разгон/круиз/торможение)
- DDA с fixed-point арифметикой (детерминизм на embedded)
- Контроль обратной связи (энкодер, синхронизация tandem-осей)
- Мягкие лимиты, плавная остановка, аварийная остановка
- HAL через абстрактный `robo::stepper`

### Существующие ресурсы в ralgo

- `trajectory/tsdeform.h` — S-curve деформатор (режим full_spattern) — **НЕ интегрирован**
- `kinematic/kinchain.h` — расчёт чувствительностей кинематической цепи
- `robo/` — драйверы stepper, encoder, quadgen

---

## Матрица сравнения функций

| Функция | Текущее | 3D-принтер | Промышленный робот |
|---------|---------|------------|-------------------|
| Линейная интерполяция (G1) | ДА | Требуется | Требуется |
| Дуговая интерполяция (G2/G3) | НЕТ | Требуется | Требуется |
| Трапецеидальный профиль | ДА | Требуется | Частично |
| S-curve (с ограничением рывка) | Частично (tsdeform есть) | Требуется | Требуется |
| Look-ahead планирование | Частично (структура есть) | Требуется | Требуется |
| Homing (G28) | НЕТ | Требуется | Требуется |
| Концевики | НЕТ | Требуется | Требуется |
| Термоконтроль | НЕТ | Требуется | — |
| Прямая/обратная кинематика | Частично (kinchain.h) | — | Требуется |
| Input shaping | НЕТ | Требуется (Klipper) | Требуется |
| Безопасность (STO, SLS) | НЕТ | — | Обязательно |

---

## Уровень 1: Базовый 3D-принтер

### 1.1. Дуговая интерполяция G2/G3

**Сложность:** Средняя
**Зависимости:** Нет

Реализация:
- Парсинг G2/G3 с параметрами I, J, K (или R для радиуса)
- Разбиение дуги на линейные сегменты с адаптивным шагом
- Поддержка плоскостей XY, XZ, YZ (G17, G18, G19)

```cpp
// Предлагаемый интерфейс
struct arc_params {
    cnc_float_type center_offset[2];  // I, J или I, K или J, K
    cnc_float_type radius;            // Альтернатива: параметр R
    bool clockwise;                   // G2=true, G3=false
    int plane;                        // 0=XY, 1=XZ, 2=YZ
};

void command_arc_move(const arc_params& params, cnc_float_type feed);
```

### 1.2. Расширение парсера G-code

**Сложность:** Низкая
**Зависимости:** Нет

Команды для добавления:
- `G0` — быстрое перемещение (использовать max velocity вместо feed)
- `G28` — последовательность поиска нуля
- `G90/G91` — абсолютный/относительный режим позиционирования
- `G92` — установка позиции (смещение системы координат)
- `M17/M18` — включение/отключение моторов

```cpp
// Добавить в класс interpreter
bool absolute_mode = true;  // G90 по умолчанию

void g_command(const nos::argv &argv, nos::ostream &os) {
    int cmd = atoi(&argv[0].data()[1]);
    switch (cmd) {
    case 0:  command_rapid_move(argv.without(1), os); break;
    case 1:  command_incremental_move(argv.without(1), os); break;
    case 2:  command_arc_cw(argv.without(1), os); break;
    case 3:  command_arc_ccw(argv.without(1), os); break;
    case 28: command_homing(argv.without(1), os); break;
    case 90: absolute_mode = true; break;
    case 91: absolute_mode = false; break;
    case 92: command_set_position(argv.without(1), os); break;
    // ...
    }
}
```

### 1.3. Look-ahead планирование

**Сложность:** Высокая
**Зависимости:** Нет

Сейчас `start_velocity = 0` и `final_velocity = 0` всегда. Нужно:

1. **Алгоритм junction deviation** (как в Marlin/Grbl):
```cpp
// Расчёт максимальной скорости на стыке двух блоков
cnc_float_type junction_velocity(
    const planner_block& prev,
    const planner_block& next,
    cnc_float_type junction_deviation)
{
    // Вычисляем угол между векторами направления
    cnc_float_type cos_theta = dot(prev.direction(), next.direction());

    // Скорость на стыке на основе центростремительного ускорения
    cnc_float_type sin_theta_d2 = sqrt(0.5 * (1.0 - cos_theta));
    return sqrt(junction_deviation * acceleration / sin_theta_d2);
}
```

2. **Обратный проход** для пересчёта скоростей:
```cpp
void recalculate_velocities() {
    // Обратный проход: убедиться, что торможение возможно
    for (int i = blocks.size() - 2; i >= 0; --i) {
        cnc_float_type max_exit = sqrt(
            blocks[i+1].start_velocity * blocks[i+1].start_velocity +
            2 * blocks[i].acceleration * blocks[i].fullpath
        );
        blocks[i].final_velocity = min(blocks[i].final_velocity, max_exit);
    }

    // Прямой проход: убедиться, что разгон возможен
    for (int i = 1; i < blocks.size(); ++i) {
        cnc_float_type max_entry = sqrt(
            blocks[i-1].final_velocity * blocks[i-1].final_velocity +
            2 * blocks[i].acceleration * blocks[i].fullpath
        );
        blocks[i].start_velocity = min(blocks[i].start_velocity, max_entry);
    }
}
```

### 1.4. Последовательность Homing

**Сложность:** Средняя
**Зависимости:** Интерфейс концевиков

```cpp
// Абстрактный интерфейс концевика
class endstop {
public:
    virtual bool is_triggered() = 0;
    virtual void enable_interrupt(igris::delegate<void> callback) = 0;
};

// Параметры homing для каждой оси
struct homing_config {
    endstop* switch_ptr;
    cnc_float_type fast_speed;      // Начальная скорость подхода
    cnc_float_type slow_speed;      // Финальная скорость подхода
    cnc_float_type backoff;         // Расстояние отката после срабатывания
    cnc_float_type offset;          // Значение позиции в home
    int8_t direction;               // +1 или -1
};

// Двухфазный homing
void home_axis(int axis) {
    auto& cfg = homing_configs[axis];

    // Фаза 1: Быстрый подход до срабатывания
    move_until_endstop(axis, cfg.direction, cfg.fast_speed);

    // Фаза 2: Откат
    relative_move(axis, -cfg.direction * cfg.backoff, cfg.slow_speed);

    // Фаза 3: Медленный подход
    move_until_endstop(axis, cfg.direction, cfg.slow_speed);

    // Установка позиции
    set_position(axis, cfg.offset);
}
```

### 1.5. Концевики и лимиты

**Сложность:** Низкая
**Зависимости:** HAL

```cpp
// Добавить в interpreter
std::array<cnc_float_type, NMAX_AXES> soft_limit_min = {};
std::array<cnc_float_type, NMAX_AXES> soft_limit_max = {};
std::array<bool, NMAX_AXES> soft_limits_enabled = {};

bool check_soft_limits(const control_task& task) {
    for (int i = 0; i < total_axes; ++i) {
        if (!soft_limits_enabled[i]) continue;

        cnc_float_type target = _final_position[i] + task.poses()[i];
        if (target < soft_limit_min[i] || target > soft_limit_max[i]) {
            report_error(error_code::position_limit_exceeded, i);
            return false;
        }
    }
    return true;
}
```

---

## Уровень 2: Продвинутый 3D-принтер (уровень Klipper)

### 2.1. Интеграция S-curve из tsdeform

**Сложность:** Средняя
**Зависимости:** Нет (tsdeform уже есть)

В `tsdeform.h` уже есть `full_spattern` для S-curve. Нужно:

1. Интегрировать tsdeform в planner_block
2. Добавить параметр jerk в interpreter
3. Модифицировать DDA для переменного ускорения

```cpp
// В planner_block
trajectory_speed_deformer spddeform;
bool use_scurve = false;

void set_state_with_scurve(
    const ralgo::vector_view<cnc_float_type>& axdist,
    int axes,
    cnc_float_type velocity,
    cnc_float_type acceleration,
    cnc_float_type jerk)
{
    // Расчёт времени разгона/торможения на основе jerk
    cnc_float_type acc_time = acceleration / jerk;
    cnc_float_type acc_part = acc_time / total_time;

    tsdeform_set_speed_pattern(&spddeform, acc_part, acc_part, 0, 0, true);
    use_scurve = true;
}

cnc_float_type current_acceleration(int64_t ic) {
    if (!use_scurve) {
        return current_acceleration_trapezoidal(ic);
    }

    double t = (double)(ic - start_ic) / (block_finish_ic - start_ic);
    return acceleration * tsdeform_spdmod(&spddeform, t);
}
```

### 2.2. Input Shaping

**Сложность:** Высокая
**Зависимости:** S-curve

```cpp
// Zero Vibration (ZV) шейпер
class input_shaper {
public:
    struct impulse {
        cnc_float_type amplitude;
        cnc_float_type time_offset;  // в тиках
    };

    std::vector<impulse> impulses;

    void configure_zv(cnc_float_type frequency, cnc_float_type damping) {
        cnc_float_type omega = 2 * M_PI * frequency;
        cnc_float_type K = exp(-damping * M_PI / sqrt(1 - damping*damping));

        impulses.clear();
        impulses.push_back({1.0 / (1 + K), 0});
        impulses.push_back({K / (1 + K), M_PI / omega});
    }

    // Применить шейпер к команде ускорения
    void shape(cnc_float_type* acc_in, cnc_float_type* acc_out, int len) {
        memset(acc_out, 0, len * sizeof(cnc_float_type));
        for (auto& imp : impulses) {
            int offset = (int)(imp.time_offset * revolver_frequency);
            for (int i = 0; i < len - offset; ++i) {
                acc_out[i + offset] += imp.amplitude * acc_in[i];
            }
        }
    }
};
```

### 2.3. Pressure Advance (для экструзии)

**Сложность:** Средняя
**Зависимости:** Look-ahead

```cpp
// Компенсация давления оси E
class pressure_advance {
    cnc_float_type K = 0;  // Коэффициент pressure advance
    int e_axis = 3;        // Индекс оси экструдера

public:
    void apply(planner_block& block, cnc_float_type velocity) {
        if (K == 0) return;

        // Смещение позиции E на основе изменения скорости
        cnc_float_type v_change = block.nominal_velocity - block.start_velocity;
        block.axdist[e_axis] += K * v_change;
    }
};
```

### 2.4. Выравнивание стола (Bed Leveling)

**Сложность:** Средняя
**Зависимости:** Интерфейс Z-probe

```cpp
// Билинейная сетка выравнивания стола
class bed_mesh {
    static const int GRID_SIZE = 5;
    cnc_float_type z_values[GRID_SIZE][GRID_SIZE];
    cnc_float_type x_min, x_max, y_min, y_max;

public:
    cnc_float_type get_z_correction(cnc_float_type x, cnc_float_type y) {
        // Билинейная интерполяция
        cnc_float_type fx = (x - x_min) / (x_max - x_min) * (GRID_SIZE - 1);
        cnc_float_type fy = (y - y_min) / (y_max - y_min) * (GRID_SIZE - 1);

        int ix = (int)fx;
        int iy = (int)fy;
        cnc_float_type dx = fx - ix;
        cnc_float_type dy = fy - iy;

        return z_values[iy][ix] * (1-dx) * (1-dy)
             + z_values[iy][ix+1] * dx * (1-dy)
             + z_values[iy+1][ix] * (1-dx) * dy
             + z_values[iy+1][ix+1] * dx * dy;
    }
};
```

---

## Уровень 3: Промышленный робот

### 3.1. Интеграция кинематики

**Сложность:** Высокая
**Зависимости:** kinchain.h (уже есть)

```cpp
// Абстрактный интерфейс кинематики
class kinematics {
public:
    virtual void forward(const cnc_float_type* joints,
                        ralgo::pose3<cnc_float_type>& pose) = 0;

    virtual bool inverse(const ralgo::pose3<cnc_float_type>& pose,
                        cnc_float_type* joints,
                        const cnc_float_type* current_joints = nullptr) = 0;

    virtual void jacobian(const cnc_float_type* joints,
                         ralgo::matrix<cnc_float_type>& J) = 0;
};

// Реализация 6-DOF робота с использованием kinchain
class robot_6dof : public kinematics {
    ralgo::pose3<double> constants[7];    // DH-преобразования
    ralgo::screw3<double> locsenses[6];   // Чувствительности сочленений

public:
    void forward(const cnc_float_type* joints,
                ralgo::pose3<cnc_float_type>& pose) override {
        // Цепное умножение преобразований
    }

    bool inverse(const ralgo::pose3<cnc_float_type>& pose,
                cnc_float_type* joints,
                const cnc_float_type* seed) override {
        // Итеративная ОК с использованием Якобиана из kinematic_chain_sensivities
    }
};
```

### 3.2. Кватернионная интерполяция ориентации

**Сложность:** Средняя
**Зависимости:** Кинематика

```cpp
// SLERP интерполяция для вращений
ralgo::quat<cnc_float_type> slerp(
    const ralgo::quat<cnc_float_type>& q0,
    const ralgo::quat<cnc_float_type>& q1,
    cnc_float_type t)
{
    cnc_float_type dot = q0.dot(q1);

    // Использовать кратчайший путь
    ralgo::quat<cnc_float_type> q1_adj = dot < 0 ? -q1 : q1;
    dot = fabs(dot);

    if (dot > 0.9995) {
        // Линейная интерполяция для почти параллельных кватернионов
        return (q0 * (1-t) + q1_adj * t).normalized();
    }

    cnc_float_type theta = acos(dot);
    cnc_float_type sin_theta = sin(theta);

    return q0 * (sin((1-t)*theta) / sin_theta)
         + q1_adj * (sin(t*theta) / sin_theta);
}
```

### 3.3. Функции безопасности (IEC 61800-5-2)

**Сложность:** Высокая
**Зависимости:** HAL, отдельный контроллер безопасности

```cpp
// Интерфейс функций безопасности
class safety_controller {
public:
    // Safe Torque Off — немедленное отключение моторов
    virtual void STO() = 0;

    // Safe Stop 1 — контролируемая остановка затем STO
    virtual void SS1(cnc_float_type decel_time) = 0;

    // Safely Limited Speed — ограничение скорости
    virtual void enable_SLS(int axis, cnc_float_type max_speed) = 0;
    virtual void disable_SLS(int axis) = 0;

    // Safely Limited Position — ограничение позиции
    virtual void enable_SLP(int axis,
                           cnc_float_type min_pos,
                           cnc_float_type max_pos) = 0;

    // Статус
    virtual bool is_safe() = 0;
    virtual uint32_t get_safety_status() = 0;
};

// Мониторинг должен выполняться в отдельном hardware или высокоприоритетном ISR
class safety_monitor {
    std::array<cnc_float_type, NMAX_AXES> sls_limits;
    std::array<cnc_float_type, NMAX_AXES> slp_min;
    std::array<cnc_float_type, NMAX_AXES> slp_max;

    void check_limits() {
        for (int i = 0; i < axes; ++i) {
            cnc_float_type vel = get_velocity(i);
            cnc_float_type pos = get_position(i);

            if (fabs(vel) > sls_limits[i]) {
                trigger_STO();
            }
            if (pos < slp_min[i] || pos > slp_max[i]) {
                trigger_SS1();
            }
        }
    }
};
```

### 3.4. Полиномиальное планирование траекторий

**Сложность:** Высокая
**Зависимости:** Нет

```cpp
// Полином 5-го порядка (квинтик)
// Обеспечивает непрерывность позиции, скорости и ускорения
class quintic_trajectory {
    cnc_float_type coeffs[6];  // a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
    cnc_float_type duration;

public:
    void plan(cnc_float_type p0, cnc_float_type v0, cnc_float_type a0,
              cnc_float_type p1, cnc_float_type v1, cnc_float_type a1,
              cnc_float_type T)
    {
        duration = T;
        cnc_float_type T2 = T * T;
        cnc_float_type T3 = T2 * T;
        cnc_float_type T4 = T3 * T;
        cnc_float_type T5 = T4 * T;

        coeffs[0] = p0;
        coeffs[1] = v0;
        coeffs[2] = a0 / 2;
        coeffs[3] = (20*(p1-p0) - (8*v1+12*v0)*T - (3*a0-a1)*T2) / (2*T3);
        coeffs[4] = (30*(p0-p1) + (14*v1+16*v0)*T + (3*a0-2*a1)*T2) / (2*T4);
        coeffs[5] = (12*(p1-p0) - 6*(v1+v0)*T - (a0-a1)*T2) / (2*T5);
    }

    void evaluate(cnc_float_type t,
                  cnc_float_type& pos,
                  cnc_float_type& vel,
                  cnc_float_type& acc)
    {
        cnc_float_type t2 = t * t;
        cnc_float_type t3 = t2 * t;
        cnc_float_type t4 = t3 * t;
        cnc_float_type t5 = t4 * t;

        pos = coeffs[0] + coeffs[1]*t + coeffs[2]*t2
            + coeffs[3]*t3 + coeffs[4]*t4 + coeffs[5]*t5;

        vel = coeffs[1] + 2*coeffs[2]*t + 3*coeffs[3]*t2
            + 4*coeffs[4]*t3 + 5*coeffs[5]*t4;

        acc = 2*coeffs[2] + 6*coeffs[3]*t
            + 12*coeffs[4]*t2 + 20*coeffs[5]*t3;
    }
};
```

### 3.5. Гарантии реального времени

**Сложность:** Высокая
**Зависимости:** RTOS или RT_PREEMPT

Требования:
- Гарантированное время отклика < 1мс
- Lock-free межпоточное взаимодействие (частично есть в error_handler)
- Watchdog с детерминированным поведением
- Предварительное выделение памяти (никакого динамического выделения в RT-пути)

```cpp
// Предвыделенные пулы памяти
template<typename T, size_t N>
class rt_pool {
    std::array<T, N> storage;
    std::array<bool, N> used;

public:
    T* allocate() {
        for (size_t i = 0; i < N; ++i) {
            if (!used[i]) {
                used[i] = true;
                return &storage[i];
            }
        }
        return nullptr;  // Пул исчерпан
    }

    void deallocate(T* ptr) {
        size_t idx = ptr - storage.data();
        if (idx < N) used[idx] = false;
    }
};
```

---

## Дорожная карта реализации

### Фаза 1: Базовый 3D-принтер

1. G90/G91, G92, G0 — простые дополнения парсера
2. Software limits — защита от выхода за пределы
3. Интерфейс концевиков + G28 homing
4. Look-ahead планирование — критично для качества печати
5. G2/G3 дуговая интерполяция

### Фаза 2: Уровень Klipper

6. Интеграция S-curve из tsdeform
7. Input shaping (минимум ZV)
8. Pressure advance
9. Bed leveling

### Фаза 3: Промышленный уровень

10. Интеграция обратной кинематики
11. Кватернионная интерполяция
12. Функции безопасности (требует аппаратной поддержки)
13. Полиномиальные траектории

---

## Быстрые победы (1-2 дня на каждое)

1. **G90/G91** — добавить флаг `absolute_mode` в interpreter
2. **G92** — установка текущей позиции (уже есть `setpos` в CLI)
3. **Software limits** — проверка в `evaluate_interpreter_task`
4. **G0** — использовать `max_axes_velocities` вместо feed

---

## Стратегия тестирования

Каждая новая функция должна иметь:
1. Unit-тесты (doctest, уже используется в проекте)
2. Интеграционные тесты с симулятором
3. Hardware-in-the-loop тесты

Пример структуры теста:
```cpp
TEST_CASE("Дуговая интерполяция G2") {
    // Настройка
    interpreter interp(...);

    SUBCASE("Дуга 90 градусов") {
        interp.newline("G2 X10 Y10 I10 J0 F1000");
        // Проверить, что дуга разбита на сегменты
        // Проверить корректность конечной точки
    }

    SUBCASE("Полный круг") {
        interp.newline("G2 I10 F1000");
        // Проверить возврат в начальную позицию
    }
}
```

---

## Ссылки

- Прошивка Marlin: https://github.com/MarlinFirmware/Marlin
- Прошивка Klipper: https://github.com/Klipper3d/klipper
- GRBL: https://github.com/grbl/grbl
- LinuxCNC: https://linuxcnc.org/
- IEC 61800-5-2 (Функции безопасности)
