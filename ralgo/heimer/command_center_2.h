#ifndef HEIMER_COMMAND_CENTER_2_H
#define HEIMER_COMMAND_CENTER_2_H

#include <vector>

#include <ralgo/heimer/wishfeed.h>
#include <ralgo/heimer/wishfeed_node.h>
#include <ralgo/heimer/linear_wfnode.h>
#include <ralgo/heimer/types.h>

#include <nos/print.h>

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
		std::vector<wishfeed> wishfeeds;
		//igris::array_view<phasectr> servos;

		std::vector<wishfeed_node *> wishfeed_nodes;

		static struct command_record * _comands;

	public:
		int add_wishfeed(const char * name, int dim) 
		{
			auto sig = find_wishfeed(name);
			if (sig != nullptr)
				return -1;

			wishfeeds.emplace_back(name, dim);

			return 0;
		}

		int add_wishfeed_command(int argc, char ** argv)
		{
			(void) argc;
			(void) argv;
			return 0;
		}

		wishfeed * find_wishfeed(std::string_view name)
		{
			for (auto & wf : wishfeeds)
			{
				if (wf.name() == name)
					return &wf;
			}
			return nullptr;
		}

		StatusPair create_wishfeed_repeater(
			std::string_view name, 
			std::string_view left,
			std::string_view right) 
		{
			float buf[1] = {1};
			ralgo::matrix_view_co<heimer::real> matrix {buf, 1, 1};

			auto * repeater = new linear_wfnode(matrix);
			repeater->rename(name);

			auto * lsig = find_wishfeed(left);
			auto * rsig = find_wishfeed(right);

			if (lsig == nullptr) 
				return { Status::UNRESOLVED_wishfeed, left };

			if (rsig == nullptr) 
				return { Status::UNRESOLVED_wishfeed, right };

			auto sts = repeater->bind_wishfeeds({lsig}, {rsig});
			if ((bool) sts.status) 
				return sts;

			transformers.push_back(repeater);

			return { Status::OK, {} };
		}

		void info() 
		{
			nos::print("wishfeeds: ");
			for (unsigned int i = 0; i < wishfeeds.size() - 1; ++i) 
			{
				nos::fprint("{}, ", wishfeeds[i].name());
			}
			nos::println(wishfeeds.back().name());


			nos::print("transformers: ");
			for (unsigned int i = 0; i < wishfeed_nodes.size() - 1; ++i) 
			{
				nos::fprint("{}, ", wishfeed_nodes[i]->name());
			}
			nos::println(wishfeed_nodes.back()->name());
		}

		bool has_unsorted_wishfeeds() 
		{
			for (auto & sig : wishfeeds) 
			{
				if (sig._pos == -1)
					return true;
			}

			return false;
		}

		void sort() 
		{
			int sig_counter = 0;
			int trans_counter = 0;
			for (auto & sig : wishfeeds) 
			{
				sig._pos = -1;
			}

			for (auto & sig : wishfeeds) 
			{
				if (sig.left == nullptr)
					sig._pos = counter++;
			}

			while(has_unsorted_wishfeeds()) 
			{
				for (auto & node : wishfeed_nodes) 
				{
					if (!node.is_ordered() node.is_left_wishfeeds_sorted()) 
					{

					}
				}
			}
		}

		void onestep() 
		{

		}
	};
}

#endif