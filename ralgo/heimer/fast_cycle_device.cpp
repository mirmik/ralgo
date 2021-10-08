#include <ralgo/heimer/fast_cycle_device.h>
#include <igris/util/bug.h>

DLIST_HEAD(heimer::fast_cycle_list);

__attribute__((weak)) double heimer::fast_cycle_frequence() { BUG(); return 0; }
