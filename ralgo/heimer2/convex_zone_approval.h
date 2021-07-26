#ifndef RALGO_HEIMER_CONVEX_ZONE_APPROVAL_H
#define RALGO_HEIMER_CONVEX_ZONE_APPROVAL_H

#include <ralgo/heimer2/axisctr_approval.h>

struct convex_zone_approval
{
	struct axisctr_approval approval;

	double * table;
	int points_total;
};

__BEGIN_DECLS

void convex_zone_approval_init(struct convex_zone_approval * cza);
int convex_zone_approval_check(
	struct axisctr_approval * approval,
	struct axisctr * axctr,
	int dim,
	position_t * strt,
	position_t * fini);

__END_DECLS

#endif