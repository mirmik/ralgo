#ifndef RALGO_HEIMER_EXECUTOR_H
#define RALGO_HEIMER_EXECUTOR_H

#include <ralgo/heimer/signal_processor.h>

#if HEIMER_CROW_SUPPORT
#include <crow/pubsub/publisher.h>
#endif

namespace heimer
{
    extern bool global_protection;

    class executor_class
    {
    public:
        signal_processor **order_table = nullptr;
        int order_table_size = 0;
        int order_table_capacity = 0;

        int count_of_axstates = 0;
        bool allowed_to_execution = false;

        union union_t
        {
            uint8_t flags = 0;
            struct flags_t
            {
                uint8_t dynamic : 1;
            } f;
        } u;

#if HEIMER_CROW_SUPPORT
        crow::publisher coordinate_publisher;
#endif

    public:
        executor_class() = default;
        void set_order_table(signal_processor **order_table, int capacity,
                             int size);

        void allocate_order_table(int size);
        void append_processor(signal_processor *proc);

        int order_sort();

        int serve(disctime_t curtime);
        int feedback(disctime_t curtime);
        int exec(disctime_t curtime);

        void execute_if_allowed(disctime_t curtime);
        void exec_fast_cycle();
        void activate_process();
        void deactivate_process();

#if HEIMER_CROW_SUPPORT
        void notification_prepare(const char *theme,
                                  crow::hostaddr_view addrview);
        void notify();
#endif

        ~executor_class();
    };

    extern executor_class executor;
    bool is_device_ready_for_settings_change();
    int executor_command(int argc, char **argv, char *output, int maxsize);
}

#endif
