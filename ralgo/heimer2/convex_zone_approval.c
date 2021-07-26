#include <ralgo/heimer2/convex_zone_approval.h>
#include <stddef.h>

void convex_zone_approval_init(struct convex_zone_approval * cza)
{
	cza -> table = NULL;
	cza -> points_total = 0;
}

int convex_zone_approval_check(
    struct axisctr_approval * approval,
    struct axisctr * ctr,
    int dim,
    position_t * strt,
    position_t * fini)
{
	/*if (dim != 2)
		return -1;

	linalg::vec<P, 2> t(val[0], val[1]);
	
	int in = point2_in_polygon_d(fini, cza->table, cza->points_total);

	
	return in;*/
}