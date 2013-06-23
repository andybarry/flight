#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>

#include "../../LCM/lcmt_midi.h"
#include "../../LCM/lcmt_stereo_control.h"
#include "../../LCM/lcmt_process_control.h"

lcm_t * lcm;
char *lcm_out = NULL;
char *processChan = NULL;
lcmt_midi_subscription_t * midi_sub;

static void usage(void)
{
        fprintf(stderr, "usage: lcmmidi-plane2 input stereo process\n");
        fprintf(stderr, "    input-channel-name : LCM channel name with MIDI LCM messages\n");
        fprintf(stderr, "    stereol-control-channel-name : LCM channel name with stereo control messages\n");
        fprintf(stderr, "    process-control-channel-name : LCM channel name with process control messages\n");
        fprintf(stderr, "  example:\n");
        fprintf(stderr, "    lcmmidi-plane2 midi_out stereo_control process_control");
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
    lcmt_process_control msgProc;
    
    // default values if something unexpected is pressed
    msg2.timestamp = getTimestampNow();
    msgProc.timestamp = getTimestampNow();
    
    msgProc.paramServer = 0;
    msgProc.mavlinkLcmBridge = 0;
    msgProc.mavlinkSerial = 0;
    msgProc.stateEstimator = 0;
    msgProc.windEstimator = 0;
    msgProc.controller = 0;
    msgProc.stereo = 0;
    msgProc.logger = 0;
    
    
    int value = 2*msg->event[2];
    
    switch (msg->event[1])
    {
        case 45:
            // record button
            if (msg->event[2] == 0)
            {
                msg2.recOn = 1;
                msg2.stereoOn = 1;
                lcmt_stereo_control_publish (lcm, lcm_out, &msg2);
            }
            
            break;
        case 42:
            // stop button (the square)
            
            // only go on release
            if (msg->event[2] == 0)
            {
                msg2.recOn = 0;
                msg2.stereoOn = 0;
                lcmt_stereo_control_publish (lcm, lcm_out, &msg2);
            }
            break;
            
        case 43:
            // rewind button
            
            if (msg->event[2] == 0)
            {
                msg2.recOn = 0;
                msg2.stereoOn = 1;
                lcmt_stereo_control_publish (lcm, lcm_out, &msg2);
            }
            
            break;
            
        case 32:
            // param start
            msgProc.paramServer = 1;
            lcmt_process_control_publish (lcm, processChan, &msgProc);
            break;
            
        
        case 33:
            msgProc.mavlinkLcmBridge = 1;
            lcmt_process_control_publish (lcm, processChan, &msgProc);
            break;
        
        
        case 34:
            msgProc.mavlinkSerial = 1;
            lcmt_process_control_publish (lcm, processChan, &msgProc);
            break;
        
        case 35:
            msgProc.windEstimator = 1;
            lcmt_process_control_publish (lcm, processChan, &msgProc);
            break;
        
        case 36:
            msgProc.stateEstimator = 1;
            lcmt_process_control_publish (lcm, processChan, &msgProc);
            break;
        
        case 37:
            msgProc.controller = 1;
            lcmt_process_control_publish (lcm, processChan, &msgProc);
            break;
        
        case 38:
            msgProc.logger = 1;
            lcmt_process_control_publish (lcm, processChan, &msgProc);
            break;
            
        // --- stop buttons (the "M"s on the keypad) ---
        case 48:
            msgProc.paramServer = 2;
            lcmt_process_control_publish (lcm, processChan, &msgProc);
            break;
            
        
        case 49:
            msgProc.mavlinkLcmBridge = 2;
            lcmt_process_control_publish (lcm, processChan, &msgProc);
            break;
        
        
        case 50:
            msgProc.mavlinkSerial = 2;
            lcmt_process_control_publish (lcm, processChan, &msgProc);
            break;
        
        case 51:
            msgProc.windEstimator = 2;
            lcmt_process_control_publish (lcm, processChan, &msgProc);
            break;
        
        case 52:
            msgProc.stateEstimator = 2;
            lcmt_process_control_publish (lcm, processChan, &msgProc);
            break;
        
        case 53:
            msgProc.controller = 2;
            lcmt_process_control_publish (lcm, processChan, &msgProc);
            break;
        
        case 54:
            msgProc.logger = 2;
            lcmt_process_control_publish (lcm, processChan, &msgProc);
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
        
        if (argc!=4) {
            usage();
            exit(0);
        }
        
        lcm_in = argv[1];
        lcm_out = argv[2];
        processChan = argv[3];
        
        lcm = lcm_create ("udpm://239.255.76.67:7667?ttl=0");
        if (!lcm)
        {
            fprintf(stderr, "lcm_create for recieve failed.  Quitting.\n");
            return 1;
        }
        
        midi_sub =  lcmt_midi_subscribe (lcm, lcm_in, &lcm_midi_handler, NULL);

        
    
        struct timeval thisTime;
        
        signal(SIGINT,sighandler);

        printf("Broadcasting LCM: %s -->\n\t%s\n\t%s\n", lcm_in, lcm_out, processChan);

        while (!stop)
        {
            // read the LCM channel
            lcm_handle (lcm);
        }


        return 0;
}
