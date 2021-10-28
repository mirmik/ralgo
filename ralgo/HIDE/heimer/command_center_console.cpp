/** @file */

#include <igris/datastruct/argvc.h>
#include <ralgo/heimer/command_center_console.h>

#include <ralgo/heimer/node_types.h>

heimer::command_center_console::command_record _command_table[] = {
    {"addsignal", &heimer::command_center_console::addsignal},
    {"delsignal", &heimer::command_center_console::delsignal},
    {"addnode", &heimer::command_center_console::addnode},
    {"delnode", &heimer::command_center_console::delnode},
    {"help", &heimer::command_center_console::help}};

void heimer::command_center_console::input(char *str)
{
    char *argv[10];
    int argc = argvc_internal_split(str, argv, 10);

    for (auto &rec : _command_table)
    {
        if (strcmp(rec.str, argv[0]) == 0)
        {
            (this->*(rec.func))(argc - 1, argv + 1);
            return;
        }
    }

    printf("Command not found (use `help` for show command list)\r\n");
}

int heimer::command_center_console::addsignal(int argc, char **argv)
{
    if (argc != 2)
    {
        printf("Usage: addsignal NAME TYPEHINT");
        return -1;
    }

    const char *name = argv[0];
    const char *hint = argv[1];

    int ihint = datanode::typehint_cast(hint);
    if (ihint < 0)
    {
        printf("Wrong typehint");
        return -1;
    }

    auto *dn = _comctr->create_datanode(name, ihint);
    if (dn == nullptr)
    {
        printf("Datasignal creation error");
        return -1;
    }

    return 0;
}

int heimer::command_center_console::delsignal(int argc, char **argv)
{
    heimer::errcode sts;
    if (argc != 1)
    {
        printf("Usage: delsignal NAME");
        return -1;
    }

    const char *name = argv[0];

    if ((sts = _comctr->remove_datanode(argv[0])))
    {
        printf("%s", sts.to_string());
        return -1;
    }

    return 0;
}

int heimer::command_center_console::addnode(int argc, char **argv)
{
    if (argc != 2)
    {
        printf("Usage: addnode NAME TYPEHINT");
        return -1;
    }

    const char *name = argv[0];
    const char *hint = argv[1];

    int ihint = heimer::node_typehint_cast(hint);
    if (ihint < 0)
    {
        printf("Wrong typehint\n");
        return -1;
    }

    auto *dn = _comctr->create_node(name, ihint);
    if (dn == nullptr)
    {
        printf("Node creation error\n");
        return -1;
    }

    return 0;
}

int heimer::command_center_console::delnode(int argc, char **argv) { return 0; }

int heimer::command_center_console::help(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    printf("DataNodes managment:\n"
           "      addsignal       add signal datanode. usage: addsignal NAME "
           "TYPEHINT\n"
           "      delsignal       del signal datanode. usage: delsignal NAME\n"
           "      listsigs        print signal datanodes list\n"
           " \n"
           "Nodes managment:\n"
           "      addnode         add signal worker node. usage: addnode "
           "NODENAME TYPEHINT\n"
           "      delnode         del signal worker node. usage: delnode "
           "NODENAME\n"
           "      listnodes       print signal nodes list\n"
           "      info            print node info. usage: info NODENAME\n"
           "      nodecmd         control node. usage: nodecmd NODENAME "
           "[OPTS...]\n"
           "\n");

    return 0;
}
