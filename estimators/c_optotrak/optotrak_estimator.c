#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>

#include "../../LCM/lcmt_hotrod_optotrak.h"
#include "../../LCM/lcmt_optotrak_xhat.h"


lcm_t * lcmSend;
char *lcm_out = NULL;
double lastX[10];
double lastAngles[10];

static void usage(void)
{
        fprintf(stderr, "usage: optotrak-estimator input-channel-name output-channel-name\n\n");
}

int stop=0;

void sighandler(int dum)
{
        stop=1;
}



void lcm_hotrod_optotrak_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_hotrod_optotrak *msg, void *user)
{
	// convert message and send it via LCM
	//printf("msg: %d\n", msg->event[2]);
	
	// convert
	lcmt_optotrak_xhat msg2;
	
	int i;
	
	for (i=0;i<3;i++)
	{
	    msg2.positions[i] = msg->positions[i];
    	msg2.angles[i] = msg->angles[i];
    }
	
	
	for (i=0;i<3;i++)
	{
    	msg2.positions_dot[i] = (msg->positions[i] - lastX[i]) * 100;
    	lastX[i] = msg->positions[i];
    }
    
    for (i=0;i<3;i++)
	{
    	msg2.angles_dot[i] = (msg->angles[i] - lastAngles[i]) * 100;
    	lastAngles[i] = msg->angles[i];
    }
	
	struct timeval thisTime;
	gettimeofday(&thisTime, NULL);
    msg2.timestamp = (thisTime.tv_sec * 1000.0) + (float)thisTime.tv_usec/1000.0 + 0.5;
	
	// send via LCM
	lcmt_optotrak_xhat_publish (lcmSend, lcm_out, &msg2);
}


int main(int argc,char** argv)
{
        char *lcm_in = NULL;
        
        if (argc!=3) {
            usage();
            exit(0);
        }
        
        lcm_in = argv[1];
        lcm_out = argv[2];
        
        lcm_t * lcm;
        lcm = lcm_create ("udpm://239.255.76.67:7667?ttl=0");
        if (!lcm)
        {
            fprintf(stderr, "lcm_create for recieve failed.  Quitting.\n");
            return 1;
        }
        
        
        lcmSend = lcm_create ("udpm://239.255.76.67:7667?ttl=0");
        if (!lcmSend)
        {
            fprintf(stderr, "lcm_create for send failed.  Quitting.\n");
            return 1;
        }
        
        int i;
        // init lastX
        for (i=0;i<9;i++)
        {
            lastX[i] = 0;
            lastAngles[i] = 0;
        }
        
        lcmt_hotrod_optotrak_subscription_t * hotrod_optotrak_sub =  lcmt_hotrod_optotrak_subscribe (lcm, lcm_in, &lcm_hotrod_optotrak_handler, NULL);

        
    
        struct timeval thisTime;
        
        signal(SIGINT,sighandler);

        printf("Running estimator on input from channel \"%s\" and outputing to \"%s\"...\n", lcm_in, lcm_out);

        while (!stop)
        {
            // read the LCM channel
            lcm_handle (lcm);
        }



        printf("Closing...\n");
        
        lcmt_hotrod_u_unsubscribe (lcm, hotrod_optotrak_sub);
        lcm_destroy (lcm);

        printf("Done.\n");
        

        return 0;
}
