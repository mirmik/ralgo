/** @file */

#ifndef RALGO_HEIMER_NODE_H
#define RALGO_HEIMER_NODE_H

/**
	@file
*/

#include <igris/container/array_view.h>
#include <ralgo/heimer/errcode.h>
#include <vector>
#include <string_view>

#define NODE_NAME_MAXLEN 16

namespace heimer
{
	enum class NodeMode
	{
		SETUP,
		SHUTDOWN,
		NORMAL
	};

	enum class LoopStrategy
	{
		DIRECT,
		REVERSE
	};

	class node
	{
		char _name [NODE_NAME_MAXLEN];
		int typehint;

	public:
		void rename(const char * name);
		int name_compare(const char * name);
		virtual int doit(NodeMode mode, int argc, const int * argv) = 0;

		// Comander interface support:
		virtual heimer::errcode command(int argc, const char ** argv) 
		{
			(void) argc;
			(void) argv;
			return errcode::UNRESOLVED_COMMAND;
		}
	};

	struct node_aggregation_task
	{
		LoopStrategy strategy;
		int argc;
		int argv[3];
	};

	class node_aggregation : public node
	{
		using parent = node;

		std::vector<node *> nodes;
		std::vector<node_aggregation_task> tasks;

		void init(
			const char * name, 
			const igris::array_view<node *> nodes,
			const igris::array_view<node_aggregation_task> tasks
			);

		int direct_loop(NodeMode mode, const node_aggregation_task & task)
		{
			int sts;
			auto it = nodes.begin();
			auto eit = nodes.end();
			for (; it != eit; ++it)
			{
				sts = (*it)->doit(mode, task.argc, task.argv);
				if (sts) return sts;
			}
		}

		int reverse_loop(NodeMode mode, const node_aggregation_task & task)
		{
			int sts;
			auto it = nodes.rbegin();
			auto eit = nodes.rend();
			for (; it != eit; ++it)
			{
				sts = (*it)->doit(mode, task.argc, task.argv);
				if (sts) return sts;
			}
		}

	public:
		int doit(NodeMode mode, int argc, const int * argv) override
		{
			(void) argc;
			(void) argv;
			int sts;

			for (auto & task : tasks)
			{
				switch (task.strategy)
				{
					case LoopStrategy::DIRECT :
						if ((sts = direct_loop(mode, task)))
							return sts;
						break;

					case LoopStrategy::REVERSE :
						if ((sts = reverse_loop(mode, task)))
							return sts;
						break;

					default:
						return -1;
				}
			}

			return 0;
		}

		heimer::errcode command(int argc, const char ** argv) override
		{
			return parent::command(argc, argv);
		}
	};

	int node_typehint_cast(std::string_view str);
}

#endif
