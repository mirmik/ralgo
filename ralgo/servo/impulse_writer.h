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
		V speed;

		void write(P _target, V _speed) 
		{
			lock.lock();

			speed = _speed;
			target = _target;

			auto dst = target - current;
			auto dstsign = sign(dst);
			auto spdsign = sign(speed);
			if (dstsign != spdsign) 
			{
				speed = -speed;
				ralgo::warning("reversed speed in impulse_writer");
			}

			set_speed(_speed);

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