#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <asoundlib.h> // requires libasound2-dev (sudo apt-get install libasound2-dev)
#include <signal.h>
#include <time.h>
#include <sys/time.h>

#include "../../LCM/lcmt_midi.h"

static void usage(void)
{
        fprintf(stderr, "usage: midi-lcm device-id channel-name\n");
        fprintf(stderr, "    device-id : test ALSA input device (find them in /dev/snd/midi*)\n");
        fprintf(stderr, "    channel-name : LCM channel name to output\n");
        fprintf(stderr, "  example:\n");
        fprintf(stderr, "    midi-lcm hw:0,0 midi_out\n");
        fprintf(stderr, "    reads input on card 0, device 0, using snd_rawmidi API\n");
        fprintf(stderr, "    and outputs it on LCM channel \"midi-out\"\n");
        fprintf(stderr, "\n");
        fprintf(stderr, "Here, I'll search your available MIDI devices for you:\n---------------------------\n");

        system("amidi -l");
        fprintf(stderr, "\n---------------------------\n");

        system("ls /dev/snd/midi*");

        fprintf(stderr, "\n---------------------------\n");
}

int stop=0;
lcm_t * lcm;
snd_rawmidi_t *handle_in = 0;

void sighandler(int dum)
{
        stop=1;

        fprintf(stderr,"Closing.\n");

        if (handle_in) {
                snd_rawmidi_drain(handle_in);
                snd_rawmidi_close(handle_in);
        }

        lcm_destroy (lcm);

        exit(0);
}

int main(int argc,char** argv)
{
        int i;
        int err;
        int thru=0;
        int verbose = 0;
        char *device_in = NULL;
        char *lcm_out = NULL;



        int fd_in = -1,fd_out = -1;
        snd_rawmidi_t *handle_out = 0;

        if (argc!=3) {
            usage();
            exit(0);
        }

        device_in = argv[1];
        lcm_out = argv[2];


        lcm = lcm_create ("udpm://239.255.76.67:7667?ttl=0");
        if (!lcm)
            return 1;

        struct timeval thisTime;

        fprintf(stderr,"Using: \n");
        fprintf(stderr,"Input: ");
        fprintf(stderr,"device %s\n",device_in);
        fprintf(stderr,"Output: ");
        fprintf(stderr,"LCM channel %s\n", lcm_out);
        fprintf(stderr,"Read midi in\n");
        fprintf(stderr,"Press ctrl-c to stop\n");
        fprintf(stderr,"Broadcasting LCM: %s\n", lcm_out);


        if (device_in) {
            err = snd_rawmidi_open(&handle_in,NULL,device_in,0);
            if (err) {
                fprintf(stderr,"snd_rawmidi_open %s failed: %d\n",device_in,err);
                return -1;
            }
        }

        signal(SIGINT,sighandler);


        int num = 0;
        int val[10];

        int sysExclusive = 0;

        if (handle_in) {
            unsigned char ch;
            while (!stop) {
                snd_rawmidi_read(handle_in,&ch,1);

                // check to make sure this isn't a system exclusive message (begins with 0xF0. Ends with a 0xF7)
                if (ch == 0xF0)
                {
                    sysExclusive = 1;
                    //fprintf(stderr,"sys exclusive\n");
                }

                if (sysExclusive == 0)
                {
                    val[num] = (int)ch;
                    if (num >= 2)
                    {
                        //fprintf(stderr,"read %02x --> %02x --> %d\n", val[0], val[1], val[2]);
                        num = 0;

                        // send LCM message
                        lcmt_midi msg;

                        gettimeofday(&thisTime, NULL);
                        msg.timestamp = (thisTime.tv_sec * 1000.0) + (float)thisTime.tv_usec/1000.0 + 0.5;

                        msg.event[0] = val[0];
                        msg.event[1] = val[1];
                        msg.event[2] = val[2];

                        lcmt_midi_publish (lcm, lcm_out, &msg);


                    } else {
                        num ++;
                    }
                    //fprintf(stderr,"read %02x\n", ch);
                }

                if (ch == 0xF7)
                {
                    sysExclusive = 0;
                }
            }
        }


        fprintf(stderr,"Closing.\n");

        if (handle_in) {
                snd_rawmidi_drain(handle_in);
                snd_rawmidi_close(handle_in);
        }

        lcm_destroy (lcm);

        return 0;
}
