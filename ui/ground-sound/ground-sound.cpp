#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>

#include <iostream>
#include <string>


#include "../../LCM/lcmt_stereo_monitor.h"
#include "mav_pose_t.h" // from Fixie

#include "../../externals/ConciseArgs.hpp"

using namespace std;

char *lcm_out = NULL;

long lastSoundTime = 0;
long lastBeepTime = 0;

int last_rec_frame = -1;

lcm_t * lcm;
mav_pose_t_subscription_t * mav_pose_sub;
lcmt_stereo_monitor_subscription_t *stereo_monitor_sub;


void sighandler(int dum)
{
    printf("\nClosing... ");

    mav_pose_t_unsubscribe (lcm, mav_pose_sub);
    lcm_destroy (lcm);


    printf("done.\n");
    
    exit(0);
}

void PlaySound(string sound_str)
{
    string soundcmd = "echo \"";
    
    soundcmd = soundcmd + sound_str;
    
    soundcmd = soundcmd + "\" | festival --tts &";
    
    // play the sound
    //cout << soundcmd << endl;
    system(soundcmd.c_str());
    
}

void stereo_monitor_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_stereo_monitor *msg, void *user)
{
    // check to see if recording has started or stopped
    if (msg->frame_number > last_rec_frame && last_rec_frame > 0)
    {
        // recording continuing as normal
        
    } else {
        // recording just started
        PlaySound("video recording started");
    }
    
    last_rec_frame = msg->frame_number;
}


void pose_handler(const lcm_recv_buf_t *rbuf, const char* channel, const mav_pose_t *msg, void *user)
{
    // check to see if the object has entered or left the view
    // if it has, play a sound
        
    long thisTime = msg->utime;
    
    string soundcmd = "";

    if (lastBeepTime + (long)5000*(long)500 < thisTime)
    {
        // build the altitude command

        // convert meters to feet
        double altitudeFt = 3.28084 * msg->pos[2];
        
        
        if (altitudeFt < 0)
        {
            soundcmd = soundcmd + "altitude negative " + to_string(int(-1*altitudeFt));
        } else if (altitudeFt < 20)
        {
            soundcmd = soundcmd + "altitude " + to_string(int(altitudeFt));
        } else {
            
            soundcmd = soundcmd + to_string(int(altitudeFt)) + " feet\"";
        }
        
        PlaySound(soundcmd);
        
        lastBeepTime = thisTime;
    }    
    
}


int main(int argc,char** argv)
{
        
                
    printf("This program requires festival.  If you don't have it, sudo apt-get install festival\n");
    
    string pose_channel_str = "STATE_ESTIMATOR_POSE";
    string stereo_monitor_channel_str = "stereo_monitor";
    
    ConciseArgs parser(argc, argv);
    parser.add(pose_channel_str, "p", "pose-channel",
        "LCM channel for state estimate input");
    parser.add(stereo_monitor_channel_str, "m", "stereo-monitor-channel",
        "LCM channel for stereo monitoring messages");
    parser.parse();
    
    
    lcm = lcm_create ("udpm://239.255.76.67:7667?ttl=0");
    if (!lcm)
    {
        fprintf(stderr, "lcm_create for recieve failed.  Quitting.\n");
        return 1;
    }
    
    
    mav_pose_sub =  mav_pose_t_subscribe (lcm, pose_channel_str.c_str(),
        &pose_handler, NULL);
        
    stereo_monitor_sub =  lcmt_stereo_monitor_subscribe (lcm,
        stereo_monitor_channel_str.c_str(), &stereo_monitor_handler, NULL);

    

    signal(SIGINT,sighandler);

    printf("Listening to LCM:\n\t%s\n\t%s\n", pose_channel_str.c_str(),
        stereo_monitor_channel_str.c_str());

    while (true)
    {
        // read the LCM channel
        lcm_handle (lcm);
    }

    return 0;
}
