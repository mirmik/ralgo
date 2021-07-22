#include <stdlib.h>
#include "stdio.h"
#include "f2c.h"

#ifdef __cplusplus
extern "C" {
#endif

/* called when a subscript is out of range */

#ifdef KR_headers
extern VOID sig_die();
integer s_rnge(varn, offset, procn, line) char *varn, *procn; ftnint offset, line;
#else
extern VOID sig_die(const char*, int);
integer s_rnge(char *varn, ftnint offset, char *procn, ftnint line)
#endif
{
	abort();
	return 0;	/* not reached */
}
#ifdef __cplusplus
}
#endif
