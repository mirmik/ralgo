#ifndef HEIMER_COMMAND_CENTER_2_H
#define HEIMER_COMMAND_CENTER_2_H

#include <vector>

#include <ralgo/heimer/wishfeed.h>
#include <ralgo/heimer/wishfeed_node.h>

namespace heimer
{
	class command_center_2;

	struct command_record
	{
		const char * cmd;
		int (command_center_2::* ptr)(int argc, char ** argv);
	};

	class command_center_2
	{

		std::vector<wishfeed> signals;
		//igris::array_view<phasectr> servos;

		std::vector<wishfeed_node *> trasformers;

		static struct command_record * _comands;

	public:
		int add_signal_command(int argc, char ** argv)
		{
			return 0;
		}

		/*wishfeed_union * find_wishfeed(std::string_view name)
		{
			for (auto & wf : signals)
			{
				if (wf.name() == name)
					return &wf;
			}
			return nullptr;
		}

		void create_linear_transformer(
			const char* name;
			int dim,
			const char ** left,
			const char ** right,
			real * matrix)
		{
			wishfeed<phase> * left_signals[MAXIMUM_SIGNALS];
			wishfeed<phase> * right_signals[MAXIMUM_SIGNALS];

			for (int i = 0; i < dim; ++i)
			{
				left_signals[dim] = find_signal(left[i]);
				right_signals[dim] = find_signal(right[i]);
			}

			phase_transformer * trasformer = allocate_linear_transformer();

			transformer->rename(name);
			transformer->set_left_signals({left_signals, dim});
			transformer->set_right_signals({right_signals, dim});
			transformers.push_back(trasformer);
		}*/


		/*void create_phase_signal(
			const char * name,

		);*/


		/*int add_signal(int argc, const char ** argv)
		{

		}

		int command(int argc, const char ** argv)
		{

		}*/
	};
}

#endif