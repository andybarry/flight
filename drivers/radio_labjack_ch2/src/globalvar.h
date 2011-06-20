#ifndef GLOBALVAR_H_
#define GLOBALVAR_H_

//includes
#include <pthread.h>
#include <inttypes.h>
#include <lcm/lcm.h>
#include "../../../LCM/lcmt_hotrod_u.h"
//

//lcm xhat storage
extern lcmt_hotrod_u lcm_hotrod_u;

//mutex
extern pthread_mutex_t mutex;

int global_init();

#endif /* GLOBALVAR_H_ */
