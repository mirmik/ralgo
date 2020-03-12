#ifndef RALGO_kinematic_CHAIN_OBJECT2D_H
#define RALGO_kinematic_CHAIN_OBJECT2D_H

#include <ralgo/kinematic/link2d.h>
#include <ralgo/kinematic/chain2d.h>
//#include <ralgo/planning/htrans2_mover.h>

#include <ralgo/disctime.h>
#include <ralgo/util/backpack.h>

#include <ralgo/heimer/multiax.h>

namespace ralgo
{
	namespace heimer
	{
		template <class P, class V>
		class kin2d_controller: public virtual_multiax<P,V>
		//	virtual public ralgo::served
		{
		public:
			ralgo::kinematic_chain2d chain;
			float compensation_koefficient;

		public:

			rabbit::htrans2<float> control_position()
			{
				return chain.out()->global_location;
			}

			void setup(
			    igris::array_view<ralgo::unit2d*> a,
			    igris::array_view<ralgo::kinematic_unit2d*> b,
			    unit2d* finallink, unit2d* startlink = nullptr)
			{
				chain.setup(a, b);
				chain.collect_chain(finallink, startlink);
			}

			kin2d_controller(const char* name, axis_driver<P,V>* arr, P* poses, size_t sz)
				: virtual_multiax<P,V>(name, arr, poses, sz)
			{
				compensation_koefficient = 1 / ralgo::discrete_time_frequency();
			}

			rabbit::htrans2<float> location()
			{
				return chain.out()->global_location;
			}

			// Контроллер должен уметь восстанавливать текущее положение
			// контрлирующих элементов по реальному положению системы.
			virtual void restore_control_model() = 0;

			void update_model_location()
			{
				chain.update_model_location();
			}

			void evaluate_links_speeds(rabbit::htrans2<float> pos, rabbit::screw2<float> spd)
			{
				//TRACE();
				rabbit::screw2<float> senses[chain.pairs.size()];
				rabbit::screw2<double> dsenses[chain.pairs.size()];
				//(void) senses;
				double* spdarr = ctrspd_array();
				memset(spdarr, 0, sizeof(spdarr));

				//Можно целевую позицию переводить в связный базис, а чувствительность
				// брать по связному базису.
				// Тогда вычитание не понадобится.
				// И перевод чувствительности в основной базис не нужен будет тоже.
				chain.sensivity(senses, chain.chain[0]);
				rabbit::screw2<float> target =
				    spd + (pos - location()) * compensation_koefficient;

				//nos::println(pos, location(), pos - location(), target);

				rabbit::screw2<double> dtarget = target;

				for (unsigned int i = 0; i < chain.pairs.size(); ++i)
				{
					dsenses[i] = senses[i];
				}

				// Поиск скоростей звеньев удовлетворяющих заданному
				// управлению.
				ralgo::svd_backpack<double, rabbit::screw2<double>>(
				            spdarr, dtarget,
				            dsenses, chain.pairs.size());

				// Выставляем найденные скорости прилинкованным
				// сервам.
				//for (unsigned int i = 0; i < chain.pairs.size(); ++i)
				//{
				//	kinematic_unit2d * _unit = chain.pairs[i];
				//	unit2d_1dof * unit = (unit2d_1dof *) _unit;
				//}

				//auto axes = controlled_axes();
			}

			virtual igris::array_view<axis_driver<P,V>*> controlled_axes() = 0;

			virtual void get_control_phase(int64_t time,
			                               rabbit::htrans2<float>& pos, rabbit::screw2<float>& spd) = 0;


			virtual double* ctrspd_array() = 0;

			virtual void apply_control() = 0;

			/*void activate()
			{
				restore_control_model();
			}

			void deactivate()
			{}*/

			void serve()
			{
				if (heimer::device::controller())
				{
					rabbit::htrans2<float> pos{};
					rabbit::screw2<float> spd{};

					get_control_phase(ralgo::discrete_time(), pos, spd);
					evaluate_links_speeds(pos, spd);
					apply_control();
				}
			}
		};
	}
}

#endif