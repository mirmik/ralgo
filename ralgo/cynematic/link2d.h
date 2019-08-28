#ifndef RALGO_CYNEMATIC_LINK2D_H
#define RALGO_CYNEMATIC_LINK2D_H

#include <rabbit/space.h>
#include <ralgo/planning/phase_driver.h>

namespace ralgo
{
	class unit2d
	{
	public:
		unit2d * parent = nullptr;

		rabbit::htrans2<float> local_location;
		rabbit::htrans2<float> global_location;

	public:
		unit2d() : local_location(), global_location() {}

		void update_location()
		{
			global_location = parent->global_location * local_location;
		}

		void relocate(const rabbit::htrans2<float>& trans)
		{
			local_location = trans;
		}

		virtual bool iscynem() 
		{
			return false;	
		}
	};

	class cynematic_unit2d : public unit2d
	{
	public:
		unit2d output;

	public:
		cynematic_unit2d()
		{
			output.parent = this;
		}

		void link(unit2d* oth)
		{
			oth->parent = &output;
		}

		virtual rabbit::screw2<float> sensivity() = 0;
		virtual void read_coords() = 0;

		bool iscynem() override
		{
			return true;	
		}
	};

	class unit2d_1dof : public cynematic_unit2d
	{
		ralgo::phase_driver * phase_driver = nullptr;
		float readed_coord_multiplier = 1;

	public:
		void set_phase_driver(ralgo::phase_driver * drv)
		{
			phase_driver = drv;
		}

		virtual void set_coord(float coord) = 0;

		void set_phase_driver(ralgo::phase_driver * reader, float mul)
		{
			phase_driver = reader;
			readed_coord_multiplier = mul;
		}

		void read_coords() override
		{
			if (phase_driver == nullptr)
				return;

			float coord = phase_driver->read_coord(readed_coord_multiplier);
			set_coord(coord);
		}
	};

	class rotator2 : public unit2d_1dof
	{
		float mul;

	public:
		rotator2() : mul(1) {}
		rotator2(float mul) : mul(mul) {}

		void set_coord(float angle)
		{
			output.local_location = rabbit::htrans2<float>(angle, {0, 0});
		}

		rabbit::screw2<float> sensivity() override
		{ return { mul, {0, 0} }; }
	};

	class actuator2 : public unit2d_1dof
	{
		rabbit::vec2<float> ax;
		float mul;

	public:
		actuator2(rabbit::vec2<float> ax, float mul) : ax(ax), mul(mul) {}

		void set_coord(float angle)
		{
			output.local_location = rabbit::htrans2<float>(angle, {0, 0});
		}

		rabbit::screw2<float> sensivity() override
		{ return { 0, mul * ax }; }
	};
}

#endif