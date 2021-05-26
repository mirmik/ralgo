#ifndef HEIMER_COMMAND_CENTER_2_H
#define HEIMER_COMMAND_CENTER_2_H

namespace heimer 
{
	union wishfeed_union 
	{
		
	};

	class command_center_2 
	{
		std::vector<wishfeed_basic *> signals;
		igris::array_view<phasectr> servos;  

		std::vector<phase_transformer *> trasformers;

	public:
		wishfeed

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
		}


		void create_phase_signal(
			const char * name,

		);
	};
}

#endif