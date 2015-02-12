/*
 * Implements a Time Varying Linear Quadratic Regulator using trajectories.
 *
 * Author: Andrew Barry, <abarry@csail.mit.edu> 2015
 *
 */

#include "midi-airspeed.hpp"
#include "../../externals/ConciseArgs.hpp"

lcm_t * lcm;

lcmt_midi_subscription_t *midi_sub;
mav_altimeter_t_subscription_t *altimeter_sub;

string midi_channel = "midi-out";
string airspeed_channel = "airspeed";

double airspeed = 0;

void lcmt_midi_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_midi *msg, void *user) {

    if (msg->event[1] == 0) {

        airspeed = 30.0/127.0 * double(msg->event[2]);

    }
}


// trigger messages to be sent on altimeter receive
void mav_altimeter_t_handler(const lcm_recv_buf_t *rbuf, const char* channel, const mav_altimeter_t *msg, void *user) {

    mav_airspeed_t airspeed_msg;

    airspeed_msg.utime = GetTimestampNow();

    airspeed_msg.airspeed = airspeed;

    mav_airspeed_t_publish(lcm, airspeed_channel.c_str(), &airspeed_msg);

}

int main(int argc,char** argv) {

    bool ttl_one = false;
    string altimeter_channel = "altimeter";


    ConciseArgs parser(argc, argv);
    parser.add(ttl_one, "t", "ttl-one", "Pass to set LCM TTL=1");
    parser.add(midi_channel, "m", "midi-channel", "LCM channel to receive MIDI messages on.");
    parser.add(airspeed_channel, "a", "airspeed-channel", "LCM channel to send airspeed messages on.");
    parser.add(altimeter_channel, "l", "altimeter-channel", "LCM channel to receive altimeter messages on.");
    parser.parse();



    if (ttl_one) {
        lcm = lcm_create ("udpm://239.255.76.67:7667?ttl=1");
    } else {
        lcm = lcm_create ("udpm://239.255.76.67:7667?ttl=0");
    }

    if (!lcm)
    {
        fprintf(stderr, "lcm_create for recieve failed.  Quitting.\n");
        return 1;
    }


    midi_sub = lcmt_midi_subscribe(lcm, midi_channel.c_str(), &lcmt_midi_handler, NULL);

    altimeter_sub = mav_altimeter_t_subscribe(lcm, altimeter_channel.c_str(), &mav_altimeter_t_handler, NULL);

    // control-c handler
    signal(SIGINT,sighandler);

    printf("Receiving LCM:\n\tMIDI:%s\n\tAltimeter: %s\n\nSending LCM:\n\t%s\n", midi_channel.c_str(), altimeter_channel.c_str(), airspeed_channel.c_str());

    while (true)
    {
        // read the LCM channel
        lcm_handle (lcm);
    }

    return 0;
}


void sighandler(int dum)
{
    printf("\n\nclosing... ");

    lcmt_midi_unsubscribe(lcm, midi_sub);
    mav_altimeter_t_unsubscribe(lcm, altimeter_sub);

    lcm_destroy (lcm);

    printf("done.\n");

    exit(0);
}
