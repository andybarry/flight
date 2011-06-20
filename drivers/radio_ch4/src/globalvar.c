#include <stdio.h>
#include <stdlib.h>   
#include "globalvar.h"


lcmt_hotrod_u lcm_hotrod_u;

pthread_mutex_t mutex;

int global_init()

{

lcm_hotrod_u.elevator=0;

return 0;
}
