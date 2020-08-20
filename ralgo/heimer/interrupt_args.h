#ifndef INTERRUPT_ARGS 
#define INTERRUPT_ARGS

namespace heimer 
{
	class interrupt_args 
	{
	public:
		virtual const char * what() { return "interrupt"; }
	};

	class interrupt_args_message : public interrupt_args
	{
		const char * msg;

	public:
		interrupt_args_message(const char * str) : msg(str) {}
		const char * what() override { return msg; }	
	};
}

#endif