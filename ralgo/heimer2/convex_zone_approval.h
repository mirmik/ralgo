#ifndef RALGO_HEIMER_CONVEX_ZONE_APPROVAL_H
#define RALGO_HEIMER_CONVEX_ZONE_APPROVAL_H

#include <ralgo/heimer2/axisctr_approval.h>

struct convex_zone_approval
{
	struct axisctr_approval approval;

	double * table;
	int dim; 

	int points_total;
	int points_capacity;
};

__BEGIN_DECLS

void convex_zone_approval_init(struct convex_zone_approval * cza, int dim);

int convex_zone_approval_check(
	struct axisctr_approval * approval,
	int dim,
	position_t * strt,
	position_t * fini);

int convex_zone_approval_room(struct convex_zone_approval * cza);

void convex_zone_approval_bind_table(struct convex_zone_approval * cza, position_t * table, int cap, int size);

void convex_zone_approval_extend(struct convex_zone_approval * cza, position_t * pnt, int size);

__END_DECLS

#endif