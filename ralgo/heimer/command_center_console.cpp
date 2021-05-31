#include <ralgo/heimer/command_center_console.h>
#include <igris/datastruct/argvc.h>

void heimer::command_center_console::input(char * str) 
{
    char * argv[10];
    argvc_internal_split(str, argv, 10);

    nos::println((const char *)argv[0]);
} 



int heimer::command_center_console::addsignal(int argc, char ** argv) 
{

    return 0;
}

int heimer::command_center_console::delsignal(int argc, char ** argv) 
{

    return 0;
}

int heimer::command_center_console::addnode(int argc, char ** argv) 
{

    return 0;
}

int heimer::command_center_console::delnode(int argc, char ** argv) 
{

    return 0;
}


heimer::command_center_console::command_record _command_table[] = 
{
    { "addsignal", &heimer::command_center_console::addsignal },
    { "delsignal", &heimer::command_center_console::delsignal },
    { "addnode", &heimer::command_center_console::addnode },
    { "delnode", &heimer::command_center_console::delnode },
};

heimer::command_center_console::command_record * 
heimer::command_center_console::command_table = _command_table;