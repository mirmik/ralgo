#include <ralgo/global_protection.h>

#include <stdio.h>
#include <stdlib.h>

#include <igris/dprint.h>

volatile bool heimer::global_protection = true;

void heimer::set_global_protection(bool en) { global_protection = en; }

int heimer::set_global_protection_command(int argc, char *argv[])
{
    if (argc != 2)
    {
        printf("global_protection %d\r\n", (int)global_protection);
        return 0;
    }

    global_protection = atoi(argv[0]);

    return 0;
}
