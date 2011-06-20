// file: lmc_i.c
//
// lcm interface program.
//
#include <stdio.h>
#include <stdlib.h>    
#include "globalvar.h"
#include "lcm_i.h"
#include "main.h"

lcm_t * pLCM;
lcm_t * lcm;

void lcm_hotrod_u_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_hotrod_u *msg, void *user)
{
	lock();
	memcpy(&lcm_hotrod_u,msg,sizeof(lcmt_hotrod_u));
	unlock();
}

int lcm_get_servo()
{
   
    pLCM = lcm_create ("udpm://239.255.76.67:7667?ttl=0");
    if (!pLCM)
        return 1;

    lcmt_hotrod_u_subscription_t * hotrod_sub =  lcmt_hotrod_u_subscribe (pLCM, "hotrod_u", &lcm_hotrod_u_handler, NULL);

	while(1)
        lcm_handle (pLCM);

     lcmt_hotrod_u_unsubscribe (pLCM, hotrod_sub);
   
    lcm_destroy (pLCM);
    
    return 0;
}

void lcm_publish_u_actual(double timestamp, double aileron, double elevator, double throttle, double rudder, double kp_height, double kd_height, double kp_yaw, double kd_yaw, double kp_pitch, double kd_pitch, double kp_roll, double kd_roll )
{

/////////////// THIS CODE IS NOT USED AT THE MOMENT //////////////
    lcmt_hotrod_u msg;
   
//    msg.timestamp=(int32_t) timestamp;
    msg.timestamp= timestamp;

	/*for(k=0;k<STATESIZE;k++)
	{

	printf("%f ",u[k]);

	}

	printf("\n");*/
	
	printf("throttle: %f\n", throttle);
	
    msg.elevator = elevator;
    msg.throttle = throttle;
    msg.aileron = aileron;
    msg.rudder = rudder;
    
    msg.kp_height = kp_height;
    msg.kd_height = kd_height;
    
    msg.kp_yaw = kp_yaw;
    msg.kd_yaw = kd_yaw;
    
    msg.kp_pitch = kp_pitch;
    msg.kd_pitch = kd_pitch;
    
    msg.kp_roll = kp_roll;
    msg.kd_roll = kd_roll;
    
    
	 
    lcmt_hotrod_u_publish (lcm, "hotrod_u_actual", &msg);

}

int lcm_publish_init()
{

    lcm = lcm_create ("udpm://239.255.76.67:7667?ttl=0");
    if (!lcm)
        return 1;

return 0;

}
