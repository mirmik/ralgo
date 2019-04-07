#ifndef RALGO_IMPULSE_WRITER_H
#define RALGO_IMPULSE_WRITER_H

//#include <ralgo/servo/impulse_position_driver.h>
#include <ralgo/info.h>
#include <ralgo/util/math.h>

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
	public:
		int write(P _target, V _speed) 
		{		
			lock.lock();	
			P position_error = target_position - current_position;
			lock.unlock();

			P abserror = abs(position_error);
			if (deviation_error_enabled & abserror > maxerror) {
				dprln("control deviation error");
				return -1;
			}

			V eval_speed = _speed + position_error * alpha;
			
			if (speed_limits_enabled) 
			{
				eval_speed = ralgo::clamp(eval_speed, -maxspeed, maxspeed);
				eval_speed = ralgo::rlamp(eval_speed, minspeed);
			}

			lock.lock();
			target_position = _target;
			evaluated_speed = eval_speed;
			target_speed = _speed;
			update_worker();
			lock.unlock();
		}

	private:
		virtual void update_worker() = 0;
		
		bool deviation_error_enabled = false;
		P maxerror = 0;
		
		bool speed_limits_enabled = false;
		V minspeed = 0;
		V maxspeed = 0;		
		
	protected:
		Lock lock;

		P current_position = 0;
		P target_position = 0;
		
		V evaluated_speed;
		V target_speed = 0.0;
		V alpha = 0.005; //коэффициент компенсации ошибки по позиции
	};
}

#endif