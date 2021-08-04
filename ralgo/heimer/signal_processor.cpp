#include <ralgo/heimer/signal_processor.h>
#include <ralgo/heimer/signal.h>
#include <ralgo/log.h>

#include <igris/shell/rshell.h>
#include <igris/math.h>
#include <igris/util/bug.h>

#include <string.h>
#include <assert.h>

using namespace heimer;

DLIST_HEAD(heimer::signal_processor_list);

signal_processor::signal_processor(const char * name, int ldim, int rdim)
{
	_leftdim = ldim;
	_rightdim = rdim;
	int len = MIN(strlen(name), SIGNAL_PROCESSOR_NAME_MAX_LENGTH);
	memset(_name, 0, SIGNAL_PROCESSOR_NAME_MAX_LENGTH);
	memcpy(_name, name, len);
	dlist_add_tail(&list_lnk, &signal_processor_list);
	this->flags = 0;
}

void signal_processor::deinit()
{
	dlist_del(&list_lnk);
}

int heimer::signal_processors_count()
{
	return dlist_size(&signal_processor_list);
}

void heimer::signal_processors_list_reinit()
{
	dlist_init(&signal_processor_list);
}

int signal_processor::activate(disctime_t curtim)
{
	int success = 1;

	if (is_active())
		return 0;

	signal_head * iter = NULL;
	while ((iter = iterate_left(iter)))
	{
		int err = iter->activate(this, curtim);
		if (err)
		{
			ralgo::warn("activation is unsucess");
			success = 0;
			break;
		}
	}

	if (success)
	{
		f.active = 1;
		on_activate(curtim);
		return 0;
	}

	else
	{
		deactivate();
		return -1;
	}
}

int signal_processor::deactivate()
{
	signal_head * iter;
	int all_right_deactivated = 1;

	iter = NULL;
	while ((iter = iterate_right(iter)))
	{
		if (iter->current_controller)
			all_right_deactivated = 0;
	}

	if (!all_right_deactivated)
		return 0;

	iter = NULL;
	while ((iter = iterate_left(iter)))
	{
		int err = iter->deactivate(this);
		(void) err;
	}
	f.active = 0;

	return 0;
}

signal_processor * heimer::signal_processor_get_by_name(const char * name)
{
	signal_processor * sig;
	dlist_for_each_entry(sig, &signal_processor_list, list_lnk)
	{
		if (strncmp(sig->name().data(), name, SIGNAL_NAME_MAX_LENGTH) == 0)
			return sig;
	}
	return NULL;
}

igris::buffer signal_processor::name()
{
	return { _name, strnlen(_name, SIGNAL_PROCESSOR_NAME_MAX_LENGTH) };
}

bool signal_processor::is_active()
{
	return f.active;
}

bool signal_processor::need_activation()
{
	return f.need_activation;
}

void signal_processor::on_activate(disctime_t)
{}

void signal_processor::set_need_activation(bool en)
{
	f.need_activation = en;
}

bool signal_processor::is_dynamic_resources()
{
	return f.dynamic_resources;
}

void signal_processor::set_dynamic_resources_flag(bool en)
{
	f.dynamic_resources = en;
}

void signal_processor::release_signals()
{
	signal_head * iter;

	iter = nullptr;
	while ((iter = iterate_left(iter)))
	{
		iter->deattach_possible_controller(this);
	}

	iter = nullptr;
	while ((iter = iterate_left(iter)))
	{
		iter->deattach_listener(this);
	}
}

static
int bindleft(signal_processor * axctr, int argc, char ** argv, char * output, int outmax)
{
	if (argc != axctr->leftdim())
	{
		snprintf(output, outmax, "Can't bind %d symbols for %d _dim axisctr", argc, axctr->leftdim());
		return -1;
	}

	{
		signal_head * arr[argc];

		for (int i = 0; i < argc; ++i)
		{
			signal_head * sig = signal_get_by_name(argv[i]);

			if (!sig)
			{
				snprintf(output, outmax, "Wrong signal name '%s ' (type 'siglist' for display)", argv[i]);
				return -1;
			}

			if (sig->type != axctr->leftsigtype(i))
			{
				snprintf(output, outmax, "Wrong signal type. name:(%s)", sig->name);
				return -1;
			}

			arr[i] = sig;
		}

		axctr->set_leftside(arr);
	}

	return 0;
}

static
int bindright(signal_processor * axctr, int argc, char ** argv, char * output, int outmax)
{
	if (argc != axctr->rightdim())
	{
		snprintf(output, outmax, "Can't bind %d symbols for %d _dim axisctr", argc, axctr->rightdim());
		return -1;
	}

	{
		signal_head * arr[argc];

		for (int i = 0; i < argc; ++i)
		{
			signal_head * sig = signal_get_by_name(argv[i]);

			if (!sig)
			{
				snprintf(output, outmax, "Wrong signal name '%s' (type 'siglist' for display)", argv[i]);
				return -1;
			}

			if (sig->type != axctr->rightsigtype(i))
			{
				snprintf(output, outmax, "Wrong signal type. name:(%s)", sig->name);
				return -1;
			}

			arr[i] = sig;
		}

		axctr->set_rightside(arr);
	}

	return 0;
}

int signal_processor::command(int argc, char ** argv, char * output, int outmax)
{
	int status = ENOENT;

	if (strcmp("bindleft", argv[0]) == 0)
		status = ::bindleft(this, argc - 1, argv + 1, output, outmax);

	if (strcmp("bindright", argv[0]) == 0)
		status = ::bindright(this, argc - 1, argv + 1, output, outmax);

	return status;
}


static signal_head * _;
signal_head * signal_processor::leftsig(int i)
{
	BUG();
	return _;
}

signal_head * signal_processor::rightsig(int i)
{
	BUG();
	return _;
}

uint8_t signal_processor::leftdim()
{
	return _leftdim;
}

uint8_t signal_processor::rightdim()
{
	return _rightdim;
}

int leftsigtype(int i)
{
	BUG();
	return 0;
}

int rightsigtype(int i)
{
	BUG();
	return 0;
}

void signal_processor::set_leftside(signal_head ** arr)
{
	for (int i = 0; i < leftdim(); ++i)
	{
		set_leftsig(i, arr[i]);
		leftsig(i) -> attach_possible_controller(this);
	}
}

void signal_processor::set_rightside(signal_head ** arr)
{
	for (int i = 0; i < rightdim(); ++i)
	{
		set_rightsig(i, arr[i]);
		rightsig(i) -> attach_listener(this);
	}
}


int signal_processor::leftsigtype(int i)
{
	BUG();
	return 0;
}

int signal_processor::rightsigtype(int i)
{
	BUG();
	return 0;
}

void signal_processor::set_leftsig(int i, signal_head *)
{
	BUG();
}

void signal_processor::set_rightsig(int i, signal_head *)
{
	BUG();
}

signal_head * signal_processor::iterate_left(signal_head * iter)
{
	if (iter == NULL)
		return leftsig(0);

	for (int i = 0; i < leftdim() - 1; ++i)
	{
		if (iter == leftsig(i))
		{
			return leftsig(i+1);
		}
	}

	return NULL;
}

signal_head * signal_processor::iterate_right(signal_head * iter)
{
	if (iter == NULL)
		return rightsig(0);

	for (int i = 0; i < rightdim() - 1; ++i)
	{
		if (iter == rightsig(i))
		{
			return rightsig(i+1);
		}
	}

	return NULL;
}
