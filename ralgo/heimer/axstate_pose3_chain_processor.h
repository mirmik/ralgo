#ifndef RALGO_HEIMER_AXSTATE_POSE3_PROCESSOR_H
#define RALGO_HEIMER_AXSTATE_POSE3_PROCESSOR_H

#include <ralgo/heimer/axstate_signal_processor.h>
#include <ralgo/heimer/dof6_signal.h>
#include <ralgo/heimer/phase_signals.h>

#include <ralgo/space/pose3.h>
#include <ralgo/space/screw.h>

#include <ralgo/linalg/matrix_view.h>
#include <ralgo/linalg/vector_view.h>
#include <ralgo/linalg/svd.h>

namespace heimer
{
	class axstate_pose3_chain_settings
	{
	public:
		ralgo::pose3<double> constant_transform;
		ralgo::screw3<double> local_sensivity;
		heimer::axis_state * controlled = nullptr;
	};

	// runtime
	class axstate_pose3_chain_temporary
	{
	public:
		//ralgo::pose3<double> leftmatrix;
		//ralgo::pose3<double> rightmatrix;
		ralgo::screw3<double> result_screw;
	};

	/**
		Порядок правых осей: x y z .
	*/
	class axstate_chain3_processor : public signal_processor
	{
	public:
		ralgo::pose3<double> first_constant_transform;
		axstate_pose3_chain_settings * settings = nullptr;
		axstate_pose3_chain_temporary * temporary = nullptr;

		ralgo::screw3<velocity_t> feedback_velocity;
		ralgo::screw3<velocity_t> control_velocity;
		ralgo::pose3<position_t> feedback_position;
		ralgo::pose3<position_t> control_position;

	public:
		axstate_chain3_processor(const char * name, int leftdim);
		void set_resources(axstate_pose3_chain_settings * settings, axstate_pose3_chain_temporary * tsettings);

		void evaluate_feedback();
		ralgo::pose3<position_t> evaluate_target_position();
		ralgo::screw3<position_t> evaluate_target_velocity();
		ralgo::screw3<position_t> evaluate_position_error();

		int feedback(disctime_t time) override;
		virtual void feedback_output(const pose3<position_t>&, const screw3<velocity_t>&) = 0;

		int command(int argc, char ** argv, char * output, int outmax) override;
		void deinit() override;
		void on_activate(disctime_t) override;

		axis_state * leftax(int i);
		void set_constant(int, float, float, float, float, float, float);
		void set_sensivity(int, float, float, float, float, float, float);

		void evaluate_error();
		void evaluate_output_sensivities(ralgo::screw3<double> * sensivities);
		void backpack(ralgo::screw3<double> * sensivities);

		void allocate_resources();

		int leftsigtype(int) override { return SIGNAL_TYPE_AXIS_STATE; }
		signal_head * leftsig(int i) override { return settings[i].controlled; }
		void set_leftsig(int i, signal_head * sig) override { settings[i].controlled = static_cast<axis_state*>(sig); }
	};

	class axstate_chain3_translation_processor : public axstate_chain3_processor
	{
		heimer::phase_signal<3> * rightside;
		double  compensation_koeff = 0.1;

	public:
		axstate_chain3_translation_processor(const char * name, int leftdim)
			: axstate_chain3_processor(name, leftdim)
		{}

		void feedback_output(const pose3<position_t>&, const screw3<velocity_t>&) override
		{
			rightside->feedpos = feedback_position.lin;
			rightside->feedvel = feedback_velocity.lin;
		}

		signal_head * rightsig(int) override { return rightside; }
		int rightsigtype(int) override { return SIGNAL_TYPE_PHASE3; }
		void set_rightsig(int, signal_head * sig) override { rightside = static_cast<phase_signal<3>*>(sig) ; }

		int serve(disctime_t) override
		{
			// Строки : 3
			// Столбцов : leftdim

			auto poserr = rightside->ctrpos - feedback_position.lin;
			auto target = rightside->ctrvel + poserr * compensation_koeff;

			double Rdata[leftdim()];
			double Adata[3 * leftdim()];
			double Wdata[leftdim()];
			double Udata[3 * leftdim()];
			double Vdata[leftdim() * leftdim()];

			ralgo::matrix_view<double> A(Adata, 3, leftdim());
			ralgo::matrix_view<double> U(Udata, 3, leftdim());
			ralgo::matrix_view<double> V(Vdata, leftdim(), leftdim());
			ralgo::vector_view<double> W(Wdata, leftdim());
			ralgo::vector_view<double> R(Rdata, leftdim());
			ralgo::vector_view<double> T(&target[0], 3);

			for (int i = 0; i < leftdim(); ++i)
				for (int j = 0; j < 3; ++j) 
					A.at(j,i) = temporary[i].result_screw.lin[j];

			auto svd = ralgo::make_SVD(A, U, V, W);

			svd.solve(T, R);

			//PRINT(A);
			//PRINT(T);
			//PRINT(R);

			for (int i = 0; i < leftdim(); ++i) 
			{
				leftax(i)->ctrvel = R[i];
				leftax(i)->ctrpos = leftax(i)->feedpos;
			}
			return 0;
		}
	};
}

#endif
#include <ralgo/linalg/matrix_view.h>