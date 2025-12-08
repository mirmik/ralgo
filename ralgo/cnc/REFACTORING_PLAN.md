# План рефакторинга CNC системы единиц

## Статус: ЗАВЕРШЕНО

Все фазы рефакторинга реализованы. Результат валидации:
- Тест "with gears=1": 9995 шагов из 10000 ожидаемых (99.95% точность)
- Все 10 unit-тестов revolver проходят

## Целевая архитектура

```
G-code (mm, mm/min)
        │
        ▼
   ┌─────────────┐
   │ Interpreter │  ← Преобразование mm → steps (единственное место!)
   │             │    steps_per_unit[axis]
   └─────────────┘
        │
        ▼ (steps, steps/sec, steps/sec²)
   ┌─────────────┐
   │   Planner   │  ← Трапецеидальный профиль в ШАГАХ
   │             │    velocity: steps/sec
   │             │    acceleration: steps/sec²
   └─────────────┘
        │
        ▼ (steps/tick, steps/tick²)
   ┌─────────────┐
   │  Revolver   │  ← DDA генерация импульсов
   │    (DDA)    │    Bresenham в тактах таймера
   └─────────────┘
        │
        ▼
   [Step pulses]
```

## Фаза 1: Упрощение Revolver (DDA)

### 1.1 Убрать gears из Revolver
- Revolver не должен знать о преобразовании единиц
- Входные данные уже в steps/tick

### 1.2 Упростить fixed-point
- Уменьшить FIXED_POINT_MUL: 1.6e9 → 2^24 (16777216)
- Это даёт ~7 бит дробной части для субшаговой точности
- Ускорения смогут быть int32 без переполнения

### 1.3 Новая структура revolver_task
```cpp
struct revolver_task {
    std::array<int32_t, NMAX_AXES> accelerations_fixed;  // steps/tick² * FIXED_POINT
    int32_t duration_ticks;
};
```

### 1.4 Упростить DDA логику
```cpp
void revolver::serve() {
    // velocity += acceleration
    // position_fixed += velocity
    // if (position_fixed >= FIXED_POINT_MUL) { step++; position_fixed -= FIXED_POINT_MUL; }
    // if (position_fixed < 0) { step--; position_fixed += FIXED_POINT_MUL; }
}
```

## Фаза 2: Рефакторинг Planner

### 2.1 Единицы в Planner
- velocity: steps/sec (не абстрактные единицы)
- acceleration: steps/sec²
- distance: steps

### 2.2 Преобразование в revolver единицы
Planner знает частоту вызова revolver (tick_frequency_hz):
```cpp
// При передаче в revolver:
velocity_per_tick = velocity_steps_per_sec / tick_frequency_hz;
accel_per_tick2 = acceleration_steps_per_sec2 / (tick_frequency_hz * tick_frequency_hz);
```

### 2.3 Убрать gears из Planner
- set_gears() → удалить
- get_gears() → удалить
- Вся работа с gears переносится в Interpreter

## Фаза 3: Рефакторинг Interpreter

### 3.1 Добавить конфигурацию осей
```cpp
struct axis_config {
    double steps_per_unit;      // steps/mm
    double max_velocity;        // mm/sec (для лимитов)
    double max_acceleration;    // mm/sec² (для лимитов)
};
```

### 3.2 Преобразование в точке входа
```cpp
void interpreter::command_move(std::span<double> target_mm) {
    std::array<int64_t, NMAX_AXES> target_steps;
    for (int i = 0; i < axes; i++) {
        target_steps[i] = target_mm[i] * axis_config[i].steps_per_unit;
    }
    // Далее planner работает в шагах
    planner->add_block(target_steps, velocity_steps_per_sec, accel_steps_per_sec2);
}
```

## Фаза 4: Обновление типов данных

### 4.1 Определить чёткие типы
```cpp
// В defs.h:
using steps_t = int64_t;           // Позиция в шагах
using velocity_t = double;          // steps/sec или steps/tick
using acceleration_t = double;      // steps/sec² или steps/tick²
using fixed_t = int64_t;           // Fixed-point значения

constexpr int64_t FIXED_POINT_BITS = 24;
constexpr int64_t FIXED_POINT_MUL = 1 << FIXED_POINT_BITS;  // 16777216
```

### 4.2 Обновить planner_block
```cpp
struct planner_block {
    std::array<steps_t, NMAX_AXES> delta_steps;  // Перемещение в шагах
    std::array<double, NMAX_AXES> direction;     // Нормализованный вектор

    velocity_t entry_velocity;      // steps/sec
    velocity_t cruise_velocity;     // steps/sec
    velocity_t exit_velocity;       // steps/sec
    acceleration_t acceleration;    // steps/sec²

    steps_t total_steps;            // Длина в шагах (евклидова)

    // Временные метки в тактах
    int64_t accel_until_tick;
    int64_t decel_after_tick;
    int64_t total_ticks;
};
```

## Фаза 5: Добавить tick_frequency

### 5.1 Конфигурация частоты
```cpp
class planner {
    uint32_t tick_frequency_hz = 100000;  // 100 kHz типично

    void set_tick_frequency(uint32_t hz);
};
```

### 5.2 Использование при планировании
```cpp
// Время в тактах = расстояние / скорость * частота
int64_t ticks = (steps * tick_frequency_hz) / velocity_steps_per_sec;
```

## Порядок реализации

1. **Начать с Revolver** — это изолированный компонент
   - Упростить fixed-point
   - Написать тесты для нового API

2. **Обновить Planner** — после стабилизации Revolver
   - Перевести на steps/sec единицы
   - Добавить tick_frequency

3. **Обновить Interpreter** — последний шаг
   - Добавить steps_per_unit конфигурацию
   - Преобразование mm→steps на входе

4. **Обновить тесты** — на каждом этапе

## Обратная совместимость

На переходный период можно оставить старый API с пометкой deprecated:
```cpp
[[deprecated("Use steps_per_unit in interpreter")]]
void set_gears(const std::array<double, NMAX_AXES>& gears);
```

## Оценка объёма

- Revolver: ~150 строк изменений
- Planner: ~200 строк изменений
- Interpreter: ~100 строк изменений
- Тесты: ~300 строк новых тестов
- Общий объём: ~750 строк

## Риски

1. **Потеря точности** при уменьшении FIXED_POINT_MUL
   - Митигация: 2^24 даёт достаточную субшаговую точность

2. **Переполнение int64** для длинных перемещений
   - Митигация: При 1000 steps/mm и 10 метрах = 10M steps — влезает

3. **Изменение поведения** существующего кода
   - Митигация: Поэтапный рефакторинг с тестами на каждом шаге
