# План рефакторинга CNC системы единиц

## Статус: ЗАВЕРШЕНО

Рефакторинг полностью завершён. Архитектура упрощена.
Все 49 тестов проходят.

## Целевая архитектура (упрощённая)

```
G-code (mm, mm/sec)
        │
        ▼
┌───────────────────────────────────────────────────┐
│                  Interpreter                       │
│  • Парсинг G-code и команд                        │
│  • Хранит steps_per_mm[] (gears)                  │
│  • Хранит tick_frequency                          │
│  • Конвертация: mm → steps                        │
│  • Конвертация: mm/sec → steps/tick               │
│  • Расчёт трапецеидального профиля                │
│  • Единственное место всех преобразований единиц  │
└───────────────────────────────────────────────────┘
        │
        ▼ planner_block (steps, steps/tick, steps/tick²)
┌───────────────────────────────────────────────────┐
│                    Planner                         │
│  • Очередь блоков движения                        │
│  • Управление активным блоком                     │
│  • Вычисление текущего ускорения по профилю       │
│  • НЕ знает о единицах — просто числа             │
└───────────────────────────────────────────────────┘
        │
        ▼ revolver_task (accelerations_fixed, duration_ticks)
┌───────────────────────────────────────────────────┐
│                    Revolver                        │
│  • DDA интегрирование (fixed-point)               │
│  • Генерация step-импульсов                       │
│  • НЕ знает о единицах — чистая математика        │
└───────────────────────────────────────────────────┘
        │
        ▼
   [Step pulses]
```

## Принцип: Single Point of Conversion

Все преобразования единиц происходят в **одном месте** — Interpreter.
Planner и Revolver работают с безразмерными числами.

## Текущее состояние (что сделано)

| Компонент | Состояние | Проблемы |
|-----------|-----------|----------|
| Revolver | ✅ Готов | — |
| defs.h | ✅ Готов | — |
| Interpreter | ✅ Работает | Нужно перенести gears из planner |
| Planner | ⚠️ Работает | Лишние поля: _steps_per_mm, _tick_frequency |
| planblock | ⚠️ Работает | Нужна документация единиц |

## Фаза 2: Упрощение (TODO)

### 2.1 Перенести gears в Interpreter

**Было:**
```cpp
// planner.h
std::array<cnc_float_type, NMAX_AXES> _steps_per_mm;
void set_gears(...);
auto get_gears();
```

**Станет:**
```cpp
// interpreter.h
std::array<cnc_float_type, NMAX_AXES> _steps_per_mm;
void set_steps_per_mm(int axis, cnc_float_type value);
cnc_float_type get_steps_per_mm(int axis) const;
```

### 2.2 Удалить tick_frequency из Planner

**Было:**
```cpp
// planner.h
uint32_t _tick_frequency;  // НЕ ИСПОЛЬЗУЕТСЯ
void set_tick_frequency(uint32_t hz);
```

**Станет:**
- Удалить полностью
- Interpreter уже хранит `revolver_frequency`

### 2.3 Документировать единицы в planblock

```cpp
struct planner_block {
    // === Геометрия (в steps) ===
    std::array<cnc_float_type, NMAX_AXES> axdist;  // steps
    std::array<cnc_float_type, NMAX_AXES> _direction;  // безразмерный, |d|=1
    cnc_float_type fullpath;  // steps (евклидова длина)

    // === Скорости (в steps/tick) ===
    cnc_float_type nominal_velocity;  // steps/tick
    cnc_float_type start_velocity;    // steps/tick
    cnc_float_type final_velocity;    // steps/tick
    cnc_float_type acceleration;      // steps/tick²

    // === Тайминг (в тиках) ===
    int64_t start_ic;                 // tick
    int64_t acceleration_before_ic;   // tick
    int64_t deceleration_after_ic;    // tick
    int64_t block_finish_ic;          // tick
};
```

### 2.4 Обновить CLI команды

| Команда | Было | Станет |
|---------|------|--------|
| `setgear` | planner->set_gear() | interpreter->set_steps_per_mm() |
| `gears` | planner->get_gears() | interpreter->get_steps_per_mm() |

### 2.5 Обновить feedback_guard интеграцию

feedback_guard получает gears из interpreter, не из planner.

## Порядок реализации

1. **Добавить поля в Interpreter**
   - `_steps_per_mm[]`
   - `set_steps_per_mm()`, `get_steps_per_mm()`

2. **Обновить evaluate_interpreter_task()**
   - Использовать собственные `_steps_per_mm` вместо `planner->get_gears()`

3. **Обновить CLI команды**
   - `setgear`, `gears` работают через interpreter

4. **Удалить из Planner**
   - `_steps_per_mm`
   - `_tick_frequency`
   - `set_gears()`, `get_gears()`, `set_gear()`
   - `set_tick_frequency()`, `get_tick_frequency()`

5. **Добавить документацию единиц**
   - Комментарии в planblock.h
   - Комментарии в revolver.h

## Оценка объёма

- Interpreter: +30 строк (новые поля и методы)
- Planner: -40 строк (удаление лишнего)
- planblock: +20 строк (документация)
- CLI команды: ~10 строк изменений
- **Итого: ~60 строк нетто изменений**

## Риски

1. **Обратная совместимость CLI**
   - Митигация: команды `setgear`/`gears` сохраняют семантику

2. **feedback_guard зависимость**
   - Митигация: feedback_guard уже получает gears через interpreter

## Критерий завершения

- [x] Planner не содержит _steps_per_mm и _tick_frequency
- [x] Interpreter хранит и управляет gears
- [x] Все тесты проходят (49/49)
- [x] planblock.h содержит документацию единиц

---

## История

### Фаза 2 (завершена)
- Перенесён `_steps_per_mm[]` из Planner в Interpreter
- Удалён неиспользуемый `_tick_frequency` из Planner
- CLI команды `setgear`/`gears` работают через Interpreter
- Добавлена документация единиц в planblock.h
- Принцип Single Point of Conversion реализован

### Фаза 1 (завершена)
- Revolver: упрощён до чистого DDA
- FIXED_POINT_MUL = 2^24
- Interpreter выполняет mm→steps конвертацию
- Тесты: 49 проходят
