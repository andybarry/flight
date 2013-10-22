#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>

#include <iostream>
#include <string>



#include "mav_pose_t.h" // from Fixie

using namespace std;

char *lcm_out = NULL;

long lastSoundTime = 0;
long lastBeepTime = 0;

lcm_t * lcm;
mav_pose_t_subscription_t * mav_pose_sub;

static void usage(void)
{
        fprintf(stderr, "usage: ground-sound input-channel-name\n\tex. ./ground-sound STATE_ESTIMATOR_POSE\n\n");
}

void sighandler(int dum)
{
    printf("\nClosing... ");

    mav_pose_t_unsubscribe (lcm, mav_pose_sub);
    lcm_destroy (lcm);


    printf("done.\n");
    
    exit(0);
}



void pose_handler(const lcm_recv_buf_t *rbuf, const char* channel, const mav_pose_t *msg, void *user)
{
    // check to see if the object has entered or left the view
    // if it has, play a sound
        
    int haveMarker = 0;

    long thisTime = msg->utime;

    if (lastBeepTime + (long)5000*(long)500 < thisTime)
    {
        // build the altitude command

        // convert meters to feet
        double altitudeFt = 3.28084 * msg->pos[2];
        
        if (altitudeFt < 20)
        {
            system("echo \"altitude\" | festival --tts &");
        } else {
            
            // convert to a string
            
            string soundcmd = "echo \"" + to_string(int(altitudeFt)) + " feet\" | festival --tts &";
            //sprintf(soundcmd,"echo \"%d feet\" | festival --tts", int(altitudeFt));
            
            // play the sound
            cout << soundcmd << endl;
            system(soundcmd.c_str());
        }
        
        lastBeepTime = thisTime;
    }    
    
}


int main(int argc,char** argv)
{
        
                
        printf("This program requires festival.  If you don't have it, sudo apt-get install festival\n");
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
        
        
        mav_pose_sub =  mav_pose_t_subscribe (lcm, lcm_in, &pose_handler, NULL);

        
    
        struct timeval thisTime;
        
        signal(SIGINT,sighandler);

        printf("Listening to LCM: %s\n", lcm_in, lcm_out);

        while (true)
        {
            // read the LCM channel
            lcm_handle (lcm);
        }

        return 0;
}
