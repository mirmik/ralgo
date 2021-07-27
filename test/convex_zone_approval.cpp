#include <doctest/doctest.h>
#include <ralgo/heimer/convex_zone_approval.h>

TEST_CASE("convex_zone_approval") 
{
	struct convex_zone_approval cza;
	double table[64];

	convex_zone_approval_init(&cza, 2);
	convex_zone_approval_bind_table(&cza, table, 64, 0);

	CHECK_EQ(convex_zone_approval_room(&cza), 64);

	double points[] = 
	{
		10, 10,
		10, 20,
		20, 10,
		20, 20
	};

	convex_zone_approval_extend(&cza, points, 4);

	CHECK_EQ(convex_zone_approval_room(&cza), 60);

	double strt[2] = { 0, 0 };
	double fini[2] = { 15, 15 };

	CHECK_EQ(convex_zone_approval_check(&cza.approval, 2, strt, fini), 1);
}