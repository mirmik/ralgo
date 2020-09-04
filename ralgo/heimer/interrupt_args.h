#ifndef INTERRUPT_ARGS
#define INTERRUPT_ARGS

#define HEIMER_INTERRUPT_TYPE_USER 0
#define HEIMER_INTERRUPT_TYPE_CONTROL_UPDATE 1
#define HEIMER_INTERRUPT_TYPE_FORCE_STOP 2

namespace heimer
{
	class interrupt_args
	{
	public:
		virtual const char * what()
		{
			return "interrupt";
		}

		virtual uint8_t code()
		{
			return HEIMER_INTERRUPT_TYPE_USER;
		}
	};

	class interrupt_args_message : public interrupt_args
	{
		const char * msg;

	public:
		interrupt_args_message(const char * str) : msg(str) {}

		const char * what() override
		{
			return msg;
		}
	};

	class force_stop_interrupt_args : public interrupt_args
	{
		const char * msg;

	public:
		force_stop_interrupt_args(const char * str) : msg(str) {}

		const char * what() override
		{
			return msg;
		}

		uint8_t code() override
		{
			return HEIMER_INTERRUPT_TYPE_FORCE_STOP;
		}
	};

	class control_update_interrupt_args : public interrupt_args
	{
		uint8_t code() override
		{
			return HEIMER_INTERRUPT_TYPE_CONTROL_UPDATE;
		}
	};
}

#endif