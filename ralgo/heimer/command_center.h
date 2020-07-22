#ifndef RALGO_HEIMER_COMMAND_CENTER_H
#define RALGO_HEIMER_COMMAND_CENTER_H

#include <ralgo/heimer/axisctr.h>
#include <ralgo/heimer/linintctr.h>

namespace heimer
{
	// TODO: Переработать интерфейсы, чтобы класс мог стать нешаблонным
	template <class P, class V>
	class command_center_cls
	{
		igris::array_view<        axisctr<P, V>*> axes;
		igris::array_view<linintctr_basic<P, V>*> igroups;

	public:
		command_center_cls() {}

	public:
		void attach_axes(igris::array_view<axisctr<P, V>*> axes) { this->axes = axes; }
		void attach_igroups(igris::array_view<linintctr_basic<P, V>*> igroups) { this->igroups = igroups; }

		int axcmd(int argc, char** argv)
		{
			if (strcmp(argv[1], "--list") == 0)
			{
				for (unsigned int i = 0; i < axes.size(); ++i)
				{
					nos::fprintln("{}: {}", i, axes[i]->name());
				}
				return 0;
			}

			if (strcmp(argv[1], "--feed") == 0)
			{
				for (unsigned int i = 0; i < axes.size(); ++i)
				{
					axes[i]->print_feed();
				}
				return 0;
			}

			if (argc < 3)
			{
				nos::println("usage: axcmd AXNO CMD [ARGS ...]");
				nos::println("usage: axcmd --feed");
				nos::println("usage: axcmd --list");
				return 0;
			}

			unsigned int axno = atoi32(argv[1], 10, nullptr);

			if (axno >= axes.size()) { nos::println("undefined axis"); return 0; }

			axes[axno]->command(argc - 2, argv + 2);
			return 0;


		}

		int igcmd(int argc, char** argv)
		{
			if (strcmp(argv[1], "--list") == 0)
			{
				for (unsigned int i = 0; i < igroups.size(); ++i)
				{
					nos::fprintln("{}: {}", i, igroups[i]->name());
				}
				return 0;
			}

			if (argc < 3) { nos::println("usage: igcmd AXNO CMD [ARGS ...]; igcmd --list"); return 0; }

			unsigned int igno = atoi32(argv[1],  10, nullptr);

			if (igno >= igroups.size()) { nos::println("undefined igroup"); return 0; }

			igroups[igno]->command(argc - 2, argv + 2);
			return 0;
		}


		int feed(int argc, char** argv)
		{
			for (unsigned int i = 0; i < axes.size(); ++i)
			{
				nos::print(i, ":");
				axes[i]->print_feed();
			}

			for (unsigned int i = 0; i < igroups.size(); ++i)
			{
				nos::print(i, ":");
				igroups[i]->print_feed();
			}
			return 0;
		}

	};

	int axcmd(int argc, char** argv);
	int igcmd(int argc, char** argv);

	extern heimer::command_center_cls<float, float> command_center;
	extern igris::console_command command_center_cmdtable[];
}

#endif