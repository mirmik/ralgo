#ifndef RALGO_kinematic_CHAIN_OBJECT2D_H
#define RALGO_kinematic_CHAIN_OBJECT2D_H

#include <ralgo/kinematic/link2d.h>
#include <ralgo/kinematic/chain2d.h>

#include <ralgo/disctime.h>
#include <ralgo/linalg/backpack.h>
#include <ralgo/heimer/multiax.h>

namespace heimer
{
	template <class P, class V>
	class kinematic_chain2d_controller : public control_node
	{
	public:
		ralgo::kinematic_chain2d chain;
		float compensation_koefficient = 1;

	public:
		ralgo::htrans2<float> control_position()
		{
			return chain.out()->global_location;
		}

		void setup(
		    igris::array_view<ralgo::unit2d*> a,
		    igris::array_view<ralgo::kinematic_unit2d*> b,
		    ralgo::unit2d* finallink,
		    ralgo::unit2d* startlink = nullptr
		)
		{
			chain.setup(a, b);
			chain.collect_chain(finallink, startlink);
		}

		kinematic_chain2d_controller(const char * name) :
			control_node(name)
		{}

		ralgo::htrans2<float> location()
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

		void evaluate_links_speeds(
			ralgo::htrans2<float> pos, 
			ralgo::screw2<float> spd)
		{
			//TRACE();
			ralgo::screw2<float> senses[chain.pairs.size()];
			ralgo::screw2<double> dsenses[chain.pairs.size()];
			//(void) senses;
			double* spdarr = ctrspd_array();
			memset(spdarr, 0, sizeof(double) * chain.pairs.size());

			//Можно целевую позицию переводить в связный базис, а чувствительность
			// брать по связному базису.
			// Тогда вычитание не понадобится.
			// И перевод чувствительности в основной базис не нужен будет тоже.

			chain.sensivity(senses, chain.chain[0]);
			ralgo::screw2<float> target = //spd;
			    spd + (pos - location()) * compensation_koefficient;

			ralgo::screw2<double> dtarget = target;

			for (unsigned int i = 0; i < chain.pairs.size(); ++i)
			{
				dsenses[i] = senses[i];
			}

			// Поиск скоростей звеньев удовлетворяющих заданному
			// управлению.
			ralgo::svd_backpack<double, ralgo::screw2<double>>(
			            spdarr, dtarget,
			            dsenses, chain.pairs.size());

			// Результат возвращается через spdarr.
		}


		void set_compensate(double k)
		{
			compensation_koefficient = k;
		}

		virtual
		void get_control_phase(int64_t time,
		                       ralgo::htrans2<float>& pos, ralgo::screw2<float>& spd) = 0;

		virtual
		double* ctrspd_array() = 0;

		virtual
		void apply_control() = 0;

		/*void on_activate_handle() override
		{
			restore_control_model();
		}

		void on_deactivate_handle() override
		{}*/

		void serve()
		{
			ralgo::htrans2<float> pos{};
			ralgo::screw2<float> spd{};

			get_control_phase(ralgo::discrete_time(), pos, spd);
			evaluate_links_speeds(pos, spd);
			apply_control();
		}
	};
}

#endif