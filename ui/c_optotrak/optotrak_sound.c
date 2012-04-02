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
char gotcmd[1000];
char lostcmd[1000];
char beepcmd[1000];

long lastSoundTime = 0;
long lastBeepTime = 0;

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

    long thisTime = msg->timestamp;

    if (msg->positions[0] > -100000)
    {
        // we have the maker
        haveMarker = 1;
    } else
    {
        lastBeepTime = thisTime;
    }
    
    if (haveMarker != lastHaveMarker && (thisTime - 100) > lastSoundTime )
    {
        lastSoundTime = thisTime;
        lastHaveMarker = haveMarker;
        // play a sound
        
        if (haveMarker == 1)
        {
            // play the "got marker" sound
            system(gotcmd);
            //printf("got marker\n");
        } else {
            // play the lost marker sound
            
            system(lostcmd);
            //printf("lost marker\n");
        }
    } else {
        lastHaveMarker = haveMarker;
    }
    
    // check to see if we've had the marker for 5 seconds, if so beep
    if (haveMarker == 1 && lastBeepTime + 1000 < thisTime)
    {
        lastBeepTime = thisTime;
        // play the "still have maker" sound
        system(beepcmd);
    }
    
    
}


int main(int argc,char** argv)
{
        strcpy(gotcmd, "mplayer ");
        strcat(gotcmd, argv[0]);
        strcat(gotcmd, "-gotmarker.wav < /dev/null &");
        
        strcpy(lostcmd, "mplayer ");
        strcat(lostcmd, argv[0]);
        strcat(lostcmd, "-lostmarker.wav < /dev/null &");
        
        strcpy(beepcmd, "mplayer ");
        strcat(beepcmd, argv[0]);
        strcat(beepcmd, "-beepmarker.wav < /dev/null &");
                
        printf(gotcmd);
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
