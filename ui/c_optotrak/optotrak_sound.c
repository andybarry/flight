#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>

#include "../../LCM/lcmt_hotrod_optotrak.h"
#include "../../LCM/lcmt_optotrak_xhat.h"


char *lcm_out = NULL;
double lastX[10];
double lastAngles[10];
int lastHaveMarker = 0;

static void usage(void)
{
        fprintf(stderr, "usage: optotrak-estimator input-channel-name\n\n");
}

int stop=0;

void sighandler(int dum)
{
        stop=1;
}



void lcm_hotrod_optotrak_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_hotrod_optotrak *msg, void *user)
{
    // check to see if the object has entered or left the view
    // if it has, play a sound
        
    int haveMarker = 0;
    
    if (msg->positions[0] > -100000)
    {
        // we have the maker
        haveMarker = 1;
    }
    
    if (haveMarker != lastHaveMarker)
    {
        lastHaveMarker = haveMarker;
        // play a sound
        
        if (haveMarker == 1)
        {
            // play the "got marker" sound
            
            system("mplayer gotmarker.wav < /dev/null &");
            printf("got marker\n");
        } else {
            // play the lost marker sound
            
            system("mplayer lostmarker.wav < /dev/null &");
            printf("lost marker\n");
        }
    } else {
        lastHaveMarker = haveMarker;
    }
    
    
}


int main(int argc,char** argv)
{
        printf("This program requires mplayer.  If you don't have it, sudo apt-get install mplayer\n");
        char *lcm_in = NULL;
        
        if (argc!=2) {
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

        printf("Listening to LCM: %s\n", lcm_in, lcm_out);

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
