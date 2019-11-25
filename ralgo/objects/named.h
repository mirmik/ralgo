#ifndef RALGO_OBJECTS_NAMED_H
#define RALGO_OBJECTS_NAMED_H

namespace ralgo 
{
	class named 
	{
	public:
		virtual const char* name() = 0;
		virtual void set_name(const char*) = 0;
		virtual void set_name(const char*, const char*) = 0;
		virtual void set_name(const char*, const char*, const char*) = 0;
	};

	template <int N>
	class named_buffer : public virtual named
	{
		char _name[N];

	public:
		const char* name() override
		{
			return _name;
		}

		void set_name(const char* name) override
		{
			strcpy(_name, name);
		}

		void set_name(const char* name1, const char* name2) override
		{
			sprintf(_name, "%s:%s", name1, name2);
		}

		void set_name(const char* name1, const char* name2, const char* name3) override
		{
			sprintf(_name, "%s:%s:%s", name1, name2, name3);
		}
	};
}

#endif