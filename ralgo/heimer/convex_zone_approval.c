#include <stddef.h>
#include <string.h>

#include <igris/util/member.h>

#include <ralgo/heimer/convex_zone_approval.h>
#include <ralgo/lp/point_in_hexagon.h>

int convex_zone_approval_check(
    struct axisctr_approval * approval,
    int dim,
    position_t * strt,
    position_t * fini)
{
	struct convex_zone_approval * cza = mcast_out(approval, struct convex_zone_approval, approval);
	int in = point_in_hexagon_d(cza->table, dim, cza->points_total, fini);
	return in;
}

void convex_zone_approval_init(struct convex_zone_approval * cza, int dim)
{
	axisctr_approval_init(&cza->approval, &convex_zone_approval_check);

	cza -> dim = dim;
	cza -> table = NULL;
	cza -> points_total = 0;
	cza -> points_capacity = 0;
}

int convex_zone_approval_room(struct convex_zone_approval * cza) 
{
	return cza->points_capacity - cza->points_total;
}

void convex_zone_approval_bind_table(struct convex_zone_approval * cza, position_t * table, int cap, int size) 
{
	cza -> table = table;
	cza -> points_capacity = cap;
	cza -> points_total = size;
}

void convex_zone_approval_extend(struct convex_zone_approval * cza, position_t * pnt, int size) 
{
	int room = convex_zone_approval_room(cza);
	int toload = MIN(room, size);

	memcpy(cza->table + cza->points_total * cza->dim, pnt, toload * cza->dim * sizeof(position_t));

	cza -> points_total += toload;
}