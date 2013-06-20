#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>

#include "../../LCM/lcmt_midi.h"
#include "../../LCM/lcmt_stereo_control.h"


lcm_t * lcm;
char *lcm_out = NULL;
lcmt_midi_subscription_t * midi_sub;

static void usage(void)
{
        fprintf(stderr, "usage: lcmmidi-plane2 input-channel-name stereo-control-channel-name\n");
        fprintf(stderr, "    input-channel-name : LCM channel name with MIDI LCM messages\n");
        fprintf(stderr, "    stereol-control-channel-name : LCM channel name with stereo control messages\n");
        fprintf(stderr, "  example:\n");
        fprintf(stderr, "    lcmmidi-plane2 midi_out stereo_control");
        fprintf(stderr, "    reads LCM MIDI input and converts that to control commands for the airplane\n");
        fprintf(stderr, "    and outputs it on an LCM channel\n");
}

int stop=0;

void sighandler(int dum)
{
        stop=1;
        
        printf("Closing...\n");
        
        lcmt_midi_unsubscribe (lcm, midi_sub);
        lcm_destroy (lcm);

        printf("Done.\n");
        
        exit(0);
}

int64_t getTimestampNow()
{
    struct timeval thisTime;
    gettimeofday(&thisTime, NULL);
    return (thisTime.tv_sec * 1000000.0) + (float)thisTime.tv_usec + 0.5;
}



void lcm_midi_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_midi *msg, void *user)
{
	// convert message and send it via LCM
	//printf("msg: %d\n", msg->event[2]);
	
	/*
     * Conversion:
     * 
     * Record dot: start recording
     * Stop button: stop recording and stop stereo, write recording to disk
     * Repeat button: stop recording
     * 
     */
     
    lcmt_stereo_control msg2;
    
    // default values if something unexpected is pressed
    msg2.timestamp = getTimestampNow();
    
    int value = 2*msg->event[2];
    
    switch (msg->event[1])
    {
        case 44:
            // record button
            if (msg->event[2] == 0)
            {
                msg2.recOn = 1;
                msg2.stereoOn = 1;
                lcmt_stereo_control_publish (lcm, lcm_out, &msg2);
            }
            
            break;
        case 46:
            // stop button (the square)
            
            // only go on release
            if (msg->event[2] == 0)
            {
                msg2.recOn = 0;
                msg2.stereoOn = 0;
                lcmt_stereo_control_publish (lcm, lcm_out, &msg2);
            }
            break;
            
        case 49:
            // repeat button (under the rewind button)
            
            if (msg->event[2] == 0)
            {
                msg2.recOn = 0;
                msg2.stereoOn = 1;
                lcmt_stereo_control_publish (lcm, lcm_out, &msg2);
            }
            
            
            break;
            
        
        default:
            // something else
            
            // don't send a message
            
            
            break;
    }
    
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
        
        
        lcm = lcm_create ("udpm://239.255.76.67:7667?ttl=0");
        if (!lcm)
        {
            fprintf(stderr, "lcm_create for recieve failed.  Quitting.\n");
            return 1;
        }
        
        midi_sub =  lcmt_midi_subscribe (lcm, lcm_in, &lcm_midi_handler, NULL);

        
    
        struct timeval thisTime;
        
        signal(SIGINT,sighandler);

        printf("Broadcasting LCM: %s --> %s\n", lcm_in, lcm_out);

        while (!stop)
        {
            // read the LCM channel
            lcm_handle (lcm);
        }


        return 0;
}
