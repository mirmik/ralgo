#ifndef RALGO_IMPULSE_WRITER_H
#define RALGO_IMPULSE_WRITER_H

//#include <ralgo/servo/impulse_position_driver.h>
#include <ralgo/info.h>

namespace ralgo 
{
	/** Абстрактный контроллер импульсов. Задача контроллера импульсовать
	*	выдавать надлежащее количество импульсов с надлежащей частотой
	*	с каждым отданным импульсом контроллер изменяет значение current.
	*	Если target == current, контроллер должен остановить выдачу импульсов.
	*	lock - примитив синхронизации для разрешения колизий доступа к переменной
	*	target и операции установки скорости.
	*/
	template <class Lock, class P = int64_t, class V = float>
	struct impulse_writer
	{
		Lock lock;
		P current;
		P target;
		bool worker_runned;
		bool endpoint;
		V speed;

		P maxerror;
		V minspeed;
		V maxspeed;		
		V alpha; //коэффициент компенсации ошибки по позиции

		void write(P _target, V _speed, bool endpoint) 
		{
			lock.lock();

			speed = _speed;
			target = _target;

			P dist = target - current;
			if (ralgo::abs(dist) > maxerror) {
				ralgo::fault("deviation error");
			}

			V eval_speed = speed + () * alpha;
			eval_speed = ralgo::clamp(eval_speed, -maxspeed, maxspeed);
			eval_speed = ralgo::rlamp(eval_speed, -minspeed, minspeed);
			set_speed(eval_speed);

			if (!worker_runned)
				start_worker();
			
			lock.unlock();
		}

		virtual void start_worker() = 0;
		virtual void stop_worker() = 0;
		virtual void set_speed(V spd) = 0;
	};
}

#endif