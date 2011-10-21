#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>

#include "../../LCM/lcmt_midi.h"
#include "../../LCM/lcmt_wingeron_gains.h"


lcm_t * lcmSend;
char *lcm_out = NULL;
lcmt_wingeron_gains lastMsg;

static void usage(void)
{
        fprintf(stderr, "usage: lcmmidi-plane input-channel-name output-channel-name\n");
        fprintf(stderr, "    input-channel-name : LCM channel name with MIDI LCM messages\n");
        fprintf(stderr, "    output-channel-name : LCM channel name with plane messages\n");
        fprintf(stderr, "  example:\n");
        fprintf(stderr, "    lcmmidi-plane midi_out wingeron_gains\n");
        fprintf(stderr, "    reads LCM MIDI input and converts that to gains for the airplane\n");
        fprintf(stderr, "    and outputs it on an LCM channel\n");
}

int stop=0;

void sighandler(int dum)
{
        stop=1;
}

lcmt_wingeron_gains ConvertFromMidiLcmToPlane(const lcmt_midi *msg)
{
    /* 
    *     Conversion
    *
    * Slider 1: throttle trime   
    * Slider 2: rudder trim
    * Slider 3: elevator trim
    * Slider 4: aileronLeft trim
    * Slider 5: aileronRight trim
    *
    * Slider/knob 6: p/d gains for throttle
    * Slider/knob 7: p/d gains for rudder
    * Slider/knob 8: p/d gains for elevator
    * Slider/knob 9: p/d gains for aileron
    */
    
    lcmt_wingeron_gains msg2 = lastMsg;
    
    int value = 2*msg->event[2];
    
    switch (msg->event[1])
    {
        case 2:
            // slider 1
            msg2.trimThrottle = value;
            break;
            
        case 14:
            // knob 1
            msg2.p_x = value;
            break;
        
        case 3:
            // slider 2
            msg2.trimRudder = value;
            break;
            
        case 15:
            // knob 2
            msg2.d_x = value;
            break;
            
        case 4:
            // slider 3
            msg2.trimElevator = value;
            break;
            
        case 16:
            // knob 3
            msg2.p_z = value;
            break;
            
        case 5:
            // slider 4
            msg2.trimAileronLeft = value;
            break;
            
        case 17:
            // knob 4
            msg2.d_z = value;
            break;
            
        case 6:
            // slider 5
            msg2.trimAileronRight = value;
            break;
        
        case 18:
            // knob 5
            break;
        
        case 8:
            // slider 6
            msg2.p_throttle = value;
            break;
            
        case 19:
            // knob 6
            msg2.d_throttle = value;
            break;
            
        case 9:
            // slider 7
            msg2.p_rudder = value;
            break;
            
        case 20:
            // knob 7
            msg2.d_rudder = value;
            break;
            
        case 12:
            // slider 8
            msg2.p_elevator = value;
            break;
            
        case 21:
            // knob 8
            msg2.d_elevator = value;
            break;
            
        case 13:
            // slider 9
            msg2.p_aileron = value;
            break;
            
        case 22:
            // knob 9
            msg2.d_aileron = value;
            break;
            
        case 44:
            // rec dot
            
            // only go on release
            if (msg->event[2] == 0)
            {
                if (lastMsg.live == 0)
                {
                    msg2.live = 1;
                } else {
                    msg2.live = 0;
                }
            }
            break;
            
        
        default:
            // something else
            // KILL!
            msg2.live = 0;
            break;
    }
    
    return msg2;
}

void lcm_midi_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_midi *msg, void *user)
{
	// convert message and send it via LCM
	//printf("msg: %d\n", msg->event[2]);
	
	// convert
	lcmt_wingeron_gains msg2 = ConvertFromMidiLcmToPlane(msg);
	
	struct timeval thisTime;
	gettimeofday(&thisTime, NULL);
    msg2.timestamp = (thisTime.tv_sec * 1000.0) + (float)thisTime.tv_usec/1000.0 + 0.5;
    
    lastMsg = msg2;
	
	// send via LCM
	lcmt_wingeron_gains_publish (lcmSend, lcm_out, &msg2);
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
        
        // init last message
        lastMsg.timestamp = 128;
        lastMsg.trimThrottle = 128;
        lastMsg.trimRudder = 128;
        lastMsg.trimElevator = 128;
        lastMsg.trimAileronLeft = 128;
        lastMsg.trimAileronRight = 128;
          
        lastMsg.p_throttle = 0;
        lastMsg.d_throttle = 0;
          
        lastMsg.p_rudder = 0;
        lastMsg.d_rudder = 0;
          
        lastMsg.p_elevator = 0;
        lastMsg.d_elevator = 0;
          
        lastMsg.p_aileron = 0;
        lastMsg.d_aileron = 0;

        lastMsg.p_x = 0;
        lastMsg.d_x = 0;
          
        lastMsg.p_x = 0;
        lastMsg.d_z = 0;
        
        lastMsg.live = 0;
        
        
        lcmt_midi_subscription_t * midi_sub =  lcmt_midi_subscribe (lcm, lcm_in, &lcm_midi_handler, NULL);

        
    
        struct timeval thisTime;
        
        signal(SIGINT,sighandler);

        printf("Converting MIDI LCM on channel \"%s\" to plane gains on channel \"%s\"...\n", lcm_in, lcm_out);

        while (!stop)
        {
            // read the LCM channel
            lcm_handle (lcm);
        }



        printf("Closing...\n");
        
        lcmt_hotrod_u_unsubscribe (lcm, midi_sub);
        lcm_destroy (lcm);

        printf("Done.\n");
        

        return 0;
}
