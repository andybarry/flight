#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>

#include "../../LCM/lcmt_wingeron_u.h"
#include "../../LCM/lcmt_optotrak_xhat.h"


lcm_t * lcmSend;
char *lcm_out = NULL;
double lastX[10];
double lastAngles[10];

static void usage(void)
{
        fprintf(stderr, "usage: utape-controller input-filname optotrak-xhat-channel wingeron-u-channel\n\n");
}

int stop=0;

void sighandler(int dum)
{
        stop=1;
}



void lcm_optotrak_xhat_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_optotrak_xhat *msg, void *user)
{
    // read the state of the plane and produce a utape based on the input file
    lcmt_optotrak_xhat msg2;
    
    int i;
    
    double yaw = msg->angles[2];
    double pitch = msg->angles[0];
    double roll = msg->angles[1];
    
    yaw = yaw - 0.33;
    roll = roll + 0.1;
    
    double newangles[] = {yaw, pitch, roll};
    
    for (i=0;i<3;i++)
    {
        msg2.positions[i] = msg->positions[i];
        msg2.angles[i] = newangles[i];//msg->angles[3-i];
    }
    
    
    for (i=0;i<3;i++)
    {
        msg2.positions_dot[i] = (msg->positions[i] - lastX[i]) * 100;
        lastX[i] = msg->positions[i];
    }
    
    for (i=0;i<3;i++)
    {
        msg2.angles_dot[i] = (newangles[i] - lastAngles[i]) * 100;
        lastAngles[i] = newangles[i];
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
        char *filename_in = NULL;
        
        if (argc!=4) {
            usage();
            exit(0);
        }
        
        filename_in = argv[1];
        lcm_in = argv[2];
        lcm_out = argv[3];
        
        // read the input tape file
        
        
        // create LCM receiver
        lcm_t * lcm;
        lcm = lcm_create ("udpm://239.255.76.67:7667?ttl=0");
        if (!lcm)
        {
            fprintf(stderr, "lcm_create for recieve failed.  Quitting.\n");
            return 1;
        }
        
        // create LCM sender
        lcmSend = lcm_create ("udpm://239.255.76.67:7667?ttl=0");
        if (!lcmSend)
        {
            fprintf(stderr, "lcm_create for send failed.  Quitting.\n");
            return 1;
        }
        
        // subscribe to receive channel
        lcmt_optotrak_xhat_subscription_t * optotrak_xhat_sub =  lcmt_optotrak_xhat_subscribe (lcm, lcm_in, &lcm_optotrak_xhat_handler, NULL);

        
    
        struct timeval thisTime;
        
        signal(SIGINT,sighandler);

        printf("Broadcasting LCM: %s --> %s\n", lcm_in, lcm_out);

        while (!stop)
        {
            // read the LCM channel
            lcm_handle (lcm);
        }



        printf("Closing...\n");
        
        lcmt_optotrak_xhat_unsubscribe (lcm, optotrak_xhat_sub);
        lcm_destroy (lcm);

        printf("Done.\n");
        

        return 0;
}
