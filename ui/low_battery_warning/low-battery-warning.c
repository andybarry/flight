#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>

#include "../../LCM/lcmt_battery_status.h"


char *lcm_out = NULL;

char lowbatcmd[1000];

long lastSoundTime = 0;
long lastBeepTime = 0;

lcm_t * lcm;
lcmt_battery_status_subscription_t * battery_status_sub;

static void usage(void)
{
        fprintf(stderr, "usage: low-battery-warning input-channel-name\n\n");
}

int stop=0;

void sighandler(int dum)
{
    printf("\nClosing... ");

    lcmt_battery_status_unsubscribe (lcm, battery_status_sub);
    lcm_destroy (lcm);


    printf("done.\n");
    
    exit(0);
}



void lcm_battery_status_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_battery_status *msg, void *user)
{
    // check to see if the object has entered or left the view
    // if it has, play a sound
        
    int haveMarker = 0;

    long thisTime = msg->timestamp;

    if (msg->voltage < 10.9 && lastBeepTime + 5000*1000 < thisTime)
    {
        system(lowbatcmd);
        lastBeepTime = thisTime;
    }    
    
}


int main(int argc,char** argv)
{
        strcpy(lowbatcmd, "mplayer ");
        strcat(lowbatcmd, argv[0]);
        strcat(lowbatcmd, "-battery-low-sound.wav < /dev/null &");
                
        printf("This program requires mplayer.  If you don't have it, sudo apt-get install mplayer\n");
        char *lcm_in = NULL;
        
        if (argc!=2) {
            usage();
            exit(0);
        }
        
        lcm_in = argv[1];
        lcm_out = argv[2];
        
        lcm = lcm_create ("udpm://239.255.76.67:7667?ttl=0");
        if (!lcm)
        {
            fprintf(stderr, "lcm_create for recieve failed.  Quitting.\n");
            return 1;
        }
        
        
        battery_status_sub =  lcmt_battery_status_subscribe (lcm, lcm_in, &lcm_battery_status_handler, NULL);

        
    
        struct timeval thisTime;
        
        signal(SIGINT,sighandler);

        printf("Listening to LCM: %s\n", lcm_in, lcm_out);

        while (!stop)
        {
            // read the LCM channel
            lcm_handle (lcm);
        }

        return 0;
}
