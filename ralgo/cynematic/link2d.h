#ifndef RALGO_CYNEMATIC_LINK2D_H
#define RALGO_CYNEMATIC_LINK2D_H

namespace ralgo
{
	class unit2d
	{
		unit2d * parent;

		rabbit::htrans2 local_trans;
		rabbit::htrans2 global_trans;

	public:
		unit2d() : local_trans(), global_trans() {}

		void update_location() 
		{
			global_trans = parent->global_trans * local_trans;
		}
	};

	class cynematic_unit2d : public unit2d
	{
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
	};

	class unit2d_1dof : public cynematic_unit2d
	{
	public:
		virtual void set_coord(float coord) = 0;
	};

	class rotator : public link2d_1dof
	{
		float mul;

	public:
		rotator() : mul(1) {}
		rotator(float mul) : mul(mul) {}

		void set_coord(float angle)
		{
			output.local_trans = rabbit::htrans2(angle, {0,0});
		}

		rabbit::screw2<float> sensivity() { return { mul, {0,0} }; }
	};

	class actuator 
	{
		rabbit::vec2<float> ax;
		float mul;

	public:
		actuator(rabbit::vec2<float> ax, float mul) : ax(ax), mul(mul) {}

		void set_coord(float angle)
		{
			output.local_trans = rabbit::htrans2(angle, {0,0});
		}

		rabbit::screw2<float> sensivity() { return { 0, mul * ax }; }
	
	};
}

#endif