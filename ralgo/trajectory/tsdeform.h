#ifndef RALGO_SPEED_DEFORMATION_H
#define RALGO_SPEED_DEFORMATION_H

/**
    @file

    Деформаторы скорости нужны для того,
    чтобы наложить скоростной паттерн на
    траекторию описываемую уравнением s(t).

    Деформатор работает в условных единицах:
    x0 = 0
    x3 = 1
    t0 = 0

    Для режима с фиксированной скоростью: v1 = 1
    Для режима с фиксированной временем:  t3 = 1

    Входными параметрами для деформатора являются
    относительные входные, выходные скорости, относительные
    временные интервалы участков.
    Ускорения выражаются через входные параметры.

    Деформатор скорости D следует применять как:
    u = t / (t_fini-t_strt) - время в частях.
    x = s(D_pos(u) * t)
    v = D_spd(u) \dot{s}(D_pos(u) * t)

    Для траектории в виде отрезка с постоянной скоростью:
    \dot{s(t)} = const = V
    =>
    v = D_spd(u) * V

Основная форма (без учёта S паттерна):

    NOTE:
    График нарисован для случая, когда v0 = v3 = 0,
    хотя в алгоритме v0 и v3 могут не быть равны 0.


        t0    t1                       t2    t3
----|-----|-------------------------|-----|----
                                                                                        Ускорение
    |     |                         |     |
    a  _____
    |     |                         |     |
        |     |_________________________
                                                                        |     |
        |     |                         |_____| -d

        |     |                         |     |
--->--t01--<----------t12---------->--t23--<---
        |     |                         |     |
                                                                                            Скорость
        |  v1 |_________________________| v2  |
                 /                           \
        |   / |                         | \   |
             /                               \
        | /   |                         |   \ |
    v0 /                                   \ v3
----|-----|-------------------------|-----|---
                                                                                            Положение
        |     |                         |     |

        |     |                         |    _| x3
                                                                 x2   .""
        |     |                         |"    |
                                                                     /
        |     |                       / |     |
                                                                 /
        |     |     . . . . . . . . /   |     |
                            /
        |     |  /                      |     |
                        /
        |     |/                        |     |
                 .  x1
 x0 |_.." |                         |     |
-----------------------------------------------

    Уравнения движения без учёта S паттерна:
                                                                    a[i]*t[i:i+1]**2
    x[i+1] = x[i] + v[i]*t[i:i+1] + ----------------
                                                                                 2
    v[i+1] = v[i] + a[i]*t[i:i+1]


3. Учёт S паттерна.
     При учёте S паттерна основной расчёт, приведённый выше
     не изменяется, но меняются графики участков с ускорениями.
     S строится с тем, чтобы выходные скорость и положение
     не изменялись. При этом ускорение увеличивается до двух раз
     при полном S паттерне.

     График полного S паттерна:

        t0    t0h   t1
----|-----|-----|--
     _____          Скорость изменения ускорения.
    b |     |     |
        |     |
                    |     |
        |     |_____| -b

----|-----|-----|--
                                        Ускорение.
        |           |
                a0h=2a
        |     .     |
                 / \
        |   / | \   |
             /     \
        | /   |   \ |
a0=0 /         \  a1=0
----|-----|-----|--
                                        Скорость.
        |     |     |
                            __  v1
        |  v0h .""  |
                    /
 v0 |__.."      |

----|-----|-----|
                                        Положение.
        |     |     |

        |     |     |
                             / x1
        |  x0h  _." |
    x0 ___..."
----|-----|-----|

*/

#include <assert.h>
#include <igris/compiler.h>
#include <math.h>

struct trajectory_speed_deformer
{
    double t01;
    double t23;
    double t3;

    double v0;
    double v3;
    double v1;

    double x1;
    double x2;

    int full_spattern;
    double t2;
    double x0h, x2h;
    double v0h, v2h;
    double a0h, a2h;
    double b, c;
};

typedef struct trajectory_speed_deformer tsdeform_t;

__BEGIN_DECLS

int tsdeform_is_finished(tsdeform_t *tsd, double t);

/// Вариант движения с увеличением максимальной скорости.
/// Завершение движения произойдёт точно в установленный момент,
/// Но скорость будет изменена.
/// @param t01 - доля времени, которую займёт разгон от доли скорости v0 до
/// номинала
/// @param t23 - доля времени, которую займёт торможение до доли скорости v3
/// @param v0  - доля скорости, существующая на момент начала движения.
/// @param v3  - доля скорости, планируемая на момент окончания движения.
void tsdeform_set_timestamp_pattern(
    tsdeform_t *tsd, double t01, double t23, double v0, double v3);

/// Второй вариант инициализации, когда вместо увеличения максимальной скорости
/// расширяется время довода изделия. Номинальная скорость
/// сохраняется неизменной.
/// @param t01 - доля времени, которую займёт разгон от доли скорости v0 до
/// номинала
/// @param t23 - доля времени, которую займёт торможение до доли скорости v3
/// @param v0  - доля скорости, существующая на момент начала движения.
/// @param v3  - доля скорости, планируемая на момент окончания движения.
/// @full_spattern - включение стратегии плавного изменения ускорения.
void tsdeform_set_speed_pattern(tsdeform_t *tsd,
                                double t01,
                                double t23,
                                double v0,
                                double v3,
                                int full_spattern);

/// Интеграл коэффициента деформации
double tsdeform_posmod(tsdeform_t *tsd, double t);

/// Коэффициент деформации
double tsdeform_spdmod(tsdeform_t *tsd, double t);

/// Сбросить состояние.
void tsdeform_nullify(tsdeform_t *tsd);

/// Паттерн остановки с линейным убыванием скорости.
void tsdeform_set_stop_pattern(tsdeform_t *tsd);

__END_DECLS

#endif
