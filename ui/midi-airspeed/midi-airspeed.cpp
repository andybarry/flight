/*
 * Implements fake airspeed measurements via MIDI board for bench testing.
 *
 * Author: Andrew Barry, <abarry@csail.mit.edu> 2015
 *
 */

#include "midi-airspeed.hpp"
#include "../../externals/ConciseArgs.hpp"

lcm_t * lcm_;

lcmt_midi_subscription_t *midi_sub;
mav_indexed_measurement_t_subscription_t *altimeter_sub;

std::string midi_channel = "midi-out";
std::string airspeed_channel = "airspeed";

double airspeed = 0, airspeed_r;

void lcmt_midi_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_midi *msg, void *user) {

    if (msg->event[1] == 0) {

        airspeed = 30.0/127.0 * double(msg->event[2]);

    }
}


// trigger messages to be sent on altimeter receive
void altimeter_handler(const lcm_recv_buf_t *rbuf, const char* channel, const mav_indexed_measurement_t *msg, void *user) {

    mav_indexed_measurement_t airspeed_msg;

    long msg_timestamp = GetTimestampNow();

    airspeed_msg.utime = msg_timestamp;
    airspeed_msg.state_utime = msg_timestamp;

    airspeed_msg.measured_dim = 1; // altitude only measures 1 dimension (x-axis)

    int airspeed_z_ind[1];

    Eigen::Vector3i state_estimator_index;
    state_estimator_index = eigen_utils::RigidBodyState::velocityInds();

    airspeed_z_ind[0] = state_estimator_index[0]; // measurement on the X axis (index = 0)
    airspeed_msg.z_indices = airspeed_z_ind;

    double airspeed_value[1];
    airspeed_value[0] = airspeed;
    airspeed_msg.z_effective = airspeed_value;

    airspeed_msg.measured_cov_dim = 1;

    double airspeed_cov[1];
    airspeed_cov[0] = airspeed_r;
    airspeed_msg.R_effective = airspeed_cov;

    mav_indexed_measurement_t_publish(lcm_, airspeed_channel.c_str(), &airspeed_msg);

}

int main(int argc,char** argv) {

    bool ttl_one = false;
    std::string altimeter_channel = "altimeter";


    ConciseArgs parser(argc, argv);
    parser.add(ttl_one, "t", "ttl-one", "Pass to set LCM TTL=1");
    parser.add(midi_channel, "m", "midi-channel", "LCM channel to receive MIDI messages on.");
    parser.add(airspeed_channel, "a", "airspeed-channel", "LCM channel to send airspeed messages on.");
    parser.add(altimeter_channel, "l", "altimeter-channel", "LCM channel to receive altimeter messages on.");
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


    BotParam *param = bot_param_new_from_server(lcm_, 0);
    if (param != NULL) {
        airspeed_r = bot_param_get_double_or_fail(param, "state_estimator.airspeed.r");
    } else {
        std::cerr << "Failed to get a parameter server, aborting." << std::endl;
    }

    midi_sub = lcmt_midi_subscribe(lcm_, midi_channel.c_str(), &lcmt_midi_handler, NULL);

    altimeter_sub = mav_indexed_measurement_t_subscribe(lcm_, altimeter_channel.c_str(), &altimeter_handler, NULL);

    // control-c handler
    signal(SIGINT,sighandler);

    printf("Receiving LCM:\n\tMIDI:%s\n\tAltimeter: %s\n\nSending LCM:\n\t%s\n", midi_channel.c_str(), altimeter_channel.c_str(), airspeed_channel.c_str());

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
    mav_indexed_measurement_t_unsubscribe(lcm_, altimeter_sub);

    lcm_destroy (lcm_);

    printf("done.\n");

    exit(0);
}
