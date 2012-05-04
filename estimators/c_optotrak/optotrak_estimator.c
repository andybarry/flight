#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>

#include "../../LCM/lcmt_optotrak.h"
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



void lcm_optotrak_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_optotrak *msg, void *user)
{
    // convert message and send it via LCM
    //printf("msg: %d\n", msg->event[2]);
    
    // convert
    lcmt_optotrak_xhat msg2;
    
    int i;
    
    double yaw[10];
    double pitch[10];
    double roll[10];
    
    
    //yaw = yaw - 0.33;
    //roll = roll + 0.1;
    
    // to convert aligned coordinates to plane coordinates, we switch
    // yaw and roll because optotrak has a different definition of which is which
    

    yaw[0] = msg->roll[0]; //yaw[0];
    pitch[0] = msg->pitch[0];
    roll[0] = msg->yaw[0]; //roll[0];

    double x[10];
    double y[10];
    double z[10];
    
    x[0] = msg->x[0];
    y[0] = msg->y[0];
    z[0] = msg->z[0];
    
    msg2.positions[0] = x[0];
    msg2.positions[1] = y[0];
    msg2.positions[2] = z[0];
    
    msg2.angles[0] = yaw[0];
    msg2.angles[1] = pitch[0];
    msg2.angles[2] = roll[0];
    
    
    msg2.positions_dot[0] = (x[0] - lastX[0]) * 100;
    msg2.positions_dot[1] = (y[0] - lastX[1]) * 100;
    msg2.positions_dot[2] = (z[0] - lastX[2]) * 100;
    
    lastX[0] = x[0];
    lastX[1] = y[0];
    lastX[2] = z[0];
    
    msg2.angles_dot[0] = (yaw[0] - lastAngles[0]) * 100;
    msg2.angles_dot[1] = (pitch[0] - lastAngles[1]) * 100;
    msg2.angles_dot[2] = (roll[0] - lastAngles[2]) * 100;
    
    lastAngles[0] = yaw[0];
    lastAngles[1] = pitch[0];
    lastAngles[2] = roll[0];
    
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
        
        lcmt_optotrak_subscription_t * optotrak_sub =  lcmt_optotrak_subscribe (lcm, lcm_in, &lcm_optotrak_handler, NULL);

        
    
        struct timeval thisTime;
        
        signal(SIGINT,sighandler);

        printf("Broadcasting LCM: %s --> %s\n", lcm_in, lcm_out);

        while (!stop)
        {
            // read the LCM channel
            lcm_handle (lcm);
        }



        printf("Closing...\n");
        
        lcmt_optotrak_unsubscribe (lcm, optotrak_sub);
        lcm_destroy (lcm);

        printf("Done.\n");
        

        return 0;
}
