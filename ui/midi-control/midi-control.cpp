/*
 * Outputs control messages based on the position of sliders on the MIDI board.
 *
 * Author: Andrew Barry, <abarry@csail.mit.edu> 2015
 *
 */

#include "midi-control.hpp"
#include "../../externals/ConciseArgs.hpp"

lcm_t * lcm_;

lcmt_midi_subscription_t *midi_sub;
lcmt_deltawing_u_subscription_t *servo_out_sub;

std::string midi_channel = "midi-out";
std::string deltawing_u_channel = "deltawing_u";

int elevon_l = 1500, elevon_r = 1500, throttle = 1109;

int debug_count = 0;

void lcmt_midi_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_midi *msg, void *user) {

    int value = round(6.2992 * msg->event[2]) + 1100;

    if (msg->event[1] == 1) {

        elevon_l = value;

    } else if (msg->event[1] == 2) {
        elevon_r = value;
    }
}


// trigger messages to be sent on servo_out  receive
void servo_out_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_deltawing_u *msg, void *user) {

    lcmt_deltawing_u msg_out;

    msg_out.timestamp = GetTimestampNow();

    msg_out.elevonL = elevon_l;
    msg_out.elevonR = elevon_r;

    msg_out.throttle = throttle;

    msg_out.is_autonomous = 1;
    msg_out.video_record = 0;

    lcmt_deltawing_u_publish(lcm_, deltawing_u_channel.c_str(), &msg_out);

    if (debug_count % 150 == 0) {
        debug_count = 0;

        lcmt_debug debug_msg;
        debug_msg.utime = GetTimestampNow();
        debug_msg.debug = "midi-control";

        lcmt_debug_publish(lcm_, "debug", &debug_msg);
    }
    debug_count ++;

}

int main(int argc,char** argv) {

    bool ttl_one = false;
    std::string servo_out_channel = "servo_out";


    ConciseArgs parser(argc, argv);
    parser.add(ttl_one, "t", "ttl-one", "Pass to set LCM TTL=1");
    parser.add(midi_channel, "m", "midi-channel", "LCM channel to receive MIDI messages on.");
    parser.add(deltawing_u_channel, "u", "deltawing-u-channel", "LCM channel to send deltawing-u messages on.");
    parser.add(servo_out_channel, "s", "servo-out-channel", "LCM channel to receive servo out messages on.");
    parser.parse();



    if (ttl_one) {
        lcm_ = lcm_create ("udpm://239.255.76.67:7667?ttl=1");
    } else {
        lcm_ = lcm_create ("udpm://239.255.76.67:7667?ttl=0");
    }

    if (!lcm_)
    {
        fprintf(stderr, "lcm_create for recieve failed.  Quitting.\n");
        return 1;
    }

    midi_sub = lcmt_midi_subscribe(lcm_, midi_channel.c_str(), &lcmt_midi_handler, NULL);

    servo_out_sub = lcmt_deltawing_u_subscribe(lcm_, servo_out_channel.c_str(), &servo_out_handler, NULL);

    // control-c handler
    signal(SIGINT,sighandler);

    printf("Receiving LCM:\n\tMIDI:%s\n\tServo out (for timing): %s\n\nSending LCM:\n\t%s\n", midi_channel.c_str(), servo_out_channel.c_str(), deltawing_u_channel.c_str());

    while (true)
    {
        // read the LCM channel
        lcm_handle (lcm_);
    }

    return 0;
}


void sighandler(int dum)
{
    printf("\n\nclosing... ");

    lcmt_midi_unsubscribe(lcm_, midi_sub);
    lcmt_deltawing_u_unsubscribe(lcm_, servo_out_sub);

    lcm_destroy (lcm_);

    printf("done.\n");

    exit(0);
}
