#ifndef POSITION_SIGNALER_H
#define POSITION_SIGNALER_H

namespace ralgo
{
// POSITION
	template<class T>
	class position_signaler
	{
	public:
		virtual T position() = 0;
	};

	template<class T>
	class position_signaler_store
	{
		T curpos = 0;

	public:
		void set_position_store(T value)
		{
			curpos = value;
		}

		T position() override
		{
			return curpos;
		}
	};

// SPEED
	template<class T>
	class speed_signaler
	{
	public:
		virtual T speed() = 0;
	};

	template<class T>
	class speed_signaler_store :
		public speed_signaler<T>
	{
		T curspd = 0;

	public:
		void set_speed_store(T value)
		{
			curspd = value;
		}

		T speed() override
		{
			return curspd;
		}
	};
}

#endif