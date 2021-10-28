/** @file */

#ifndef RALGO_command_CENTER_INPUT_H
#define RALGO_command_CENTER_INPUT_H

#include <ralgo/heimer/command_center_2.h>

namespace heimer
{
    class command_center_console
    {
        using command_type = int (command_center_console::*)(int argc,
                                                             char **argv);
        command_center_2 *_comctr;

    public:
        struct command_record
        {
            const char *str;
            command_type func;
        };

        command_center_console(command_center_2 *comctr) : _comctr(comctr) {}
        void input(char *str);

        int addsignal(int argc, char **argv);
        int delsignal(int argc, char **argv);

        int addnode(int argc, char **argv);
        int delnode(int argc, char **argv);

        int help(int argc, char **argv);
    };
}

#endif
