/*
 * Reads sensor values and makes (hopefully) reasonable choices about what to do
 * if the airspeed sensor appears to be failing
 *
 * Author: Andrew Barry, <abarry@csail.mit.edu> 2015
 *
 */

#include "airspeed-check.hpp"
#include "../../externals/ConciseArgs.hpp"

#define MIN_EXPECTED_MPS 0.2
#define MAX_EXPECTED_GROUND_MPS 5
#define MIN_EXPECTED_STAND_DEV 0.1
#define NUMBER_MEASUREMENTS_ROLLING 70 // about 1 second

#define MAX_GPS_AIRSPEED_DIFF_MPS 3.5

#define ALTITUDE_DIFFERENCE_TO_BE_IN_AIR_METERS 4

#define FALLBACK_AIRSPEED 16.899



lcm_t * lcm_;


mav_gps_data_t *last_gps_msg = NULL;
mav_indexed_measurement_t *last_altimeter_msg = NULL;


std::string airspeed_in_channel = "airspeed-unchecked";
std::string airspeed_out_channel = "airspeed";
std::string altimeter_channel = "altitude";
std::string gps_channel = "gps";


RollingStatistics *rolling_stats = NULL;

int debug_count = 0;

double min_altitude = -1;

mav_indexed_measurement_t_subscription_t *airspeed_in_sub;
mav_indexed_measurement_t_subscription_t *altitude_sub;
mav_gps_data_t_subscription_t *gps_sub;




void airspeed_handler(const lcm_recv_buf_t *rbuf, const char* channel, const mav_indexed_measurement_t *msg, void *user) {

    // compute statistics about airspeed to decide if it is reasonable
    double airspeed = msg->z_effective[0];

    rolling_stats->AddValue(airspeed);

    if (rolling_stats->GetMean() < MIN_EXPECTED_MPS || rolling_stats->GetStandardDeviation() < MIN_EXPECTED_STAND_DEV) {
        // potentially bad value (low mean or low stand dev)

        // check if GPS is ok
        if (last_gps_msg != NULL && last_gps_msg->gps_lock >= 3) {

            // GPS is online

            // compare GPS and airspeed
            if (abs(last_gps_msg->speed - airspeed) > MAX_GPS_AIRSPEED_DIFF_MPS) {
                // GPS and airspeed differ

                SendNewAirspeedMessage(last_gps_msg->speed, msg);
                SendDebugMessage("airspeed-check-gps");

            } else {
                // GPS and airspeed agree
                SendExistingAirspeedMessage(msg);

            }

        } else {
            // GPS offline AND potentially bad airspeed

            // check the altimeter
            if (last_altimeter_msg != NULL && abs(last_altimeter_msg->z_effective[0] - min_altitude) > ALTITUDE_DIFFERENCE_TO_BE_IN_AIR_METERS) {
                // in the air

                // if we are here, we have NO CLUE what is going on:
                //
                // 1. Airspeed looks bad
                // 2. GPS is offline
                // 3. We seem to be flying

                // so, to ensure we don't crash because the plane thinks it's going 0 m/s, we make up a
                // reasonable speed and send that as the message
                // we also send a debug message to let the user know that something bad is happening

                SendNewAirspeedMessage(FALLBACK_AIRSPEED, msg);
                SendDebugMessage("airspeed-check-fallback");

            } else {
                // on the ground -- probably nothing is wrong
                SendExistingAirspeedMessage(msg);
            }

        }

    } else {
        // airspeed looks fine
        SendExistingAirspeedMessage(msg);
    }

}

void gps_handler(const lcm_recv_buf_t *rbuf, const char* channel, const mav_gps_data_t *msg, void *user) {
    // save the lastest GPS message

    if (last_gps_msg != NULL) {
        delete last_gps_msg;
    }

    last_gps_msg = mav_gps_data_t_copy(msg);

}

void altimeter_handler(const lcm_recv_buf_t *rbuf, const char* channel, const mav_indexed_measurement_t *msg, void *user) {

    double altitude = msg->z_effective[0];

    if (altitude < min_altitude) {
        min_altitude = altitude;
    }

    if (last_altimeter_msg != NULL) {
        delete last_altimeter_msg;
    }

    last_altimeter_msg = mav_indexed_measurement_t_copy(msg);

}

void SendNewAirspeedMessage(double new_speed, const mav_indexed_measurement_t *old_msg) {

    mav_indexed_measurement_t airspeed_msg;

    int64_t msg_timestamp;
    msg_timestamp = GetTimestampNow();

    airspeed_msg.utime = msg_timestamp;
    airspeed_msg.state_utime = msg_timestamp;

    double airspeed = new_speed;

    airspeed_msg.measured_dim = 1; // altitude only measures 1 dimension (x-axis)

    Eigen::Vector3i state_estimator_index;

    int airspeed_z_ind[1];
    state_estimator_index = eigen_utils::RigidBodyState::velocityInds();
    airspeed_z_ind[0] = state_estimator_index[0]; // measurement on the X axis (index = 0)
    airspeed_msg.z_indices = airspeed_z_ind;

    double airspeed_value[1];
    airspeed_value[0] = airspeed;
    airspeed_msg.z_effective = airspeed_value;

    airspeed_msg.measured_cov_dim = 1;

    double airspeed_cov[1];
    airspeed_cov[0] = old_msg->R_effective[0];
    airspeed_msg.R_effective = airspeed_cov;

    mav_indexed_measurement_t_publish(lcm_, airspeed_out_channel.c_str(), &airspeed_msg);

}

void SendExistingAirspeedMessage(const mav_indexed_measurement_t *msg) {
    mav_indexed_measurement_t_publish(lcm_, airspeed_out_channel.c_str(), msg);
}

void SendDebugMessage(char *debug_str) {
    if (debug_count % 70 == 0) {
        debug_count = 0;

        lcmt_debug debug_msg;
        debug_msg.utime = GetTimestampNow();
        debug_msg.debug = debug_str;

        lcmt_debug_publish(lcm_, "debug", &debug_msg);
    }
    debug_count ++;
}


int main(int argc,char** argv) {

    bool ttl_one = false;


    ConciseArgs parser(argc, argv);
    parser.add(ttl_one, "t", "ttl-one", "Pass to set LCM TTL=1");
    parser.add(airspeed_in_channel, "i", "airspeed-in-channel", "LCM channel to receive airspeed messages on.");
    parser.add(airspeed_out_channel, "o", "airspeed-out-channel", "LCM channel to send corrected airspeed messages on.");
    parser.add(altimeter_channel, "a", "altimeter-channel", "LCM channel to receive altitude messages on.");
    parser.add(gps_channel, "g", "gps-channel", "LCM channel to receive GPS messages on.");
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

    rolling_stats = new RollingStatistics(NUMBER_MEASUREMENTS_ROLLING);

    airspeed_in_sub = mav_indexed_measurement_t_subscribe(lcm_, airspeed_in_channel.c_str(), &airspeed_handler, NULL);
    altitude_sub = mav_indexed_measurement_t_subscribe(lcm_, altimeter_channel.c_str(), &altimeter_handler, NULL);
    gps_sub = mav_gps_data_t_subscribe(lcm_, gps_channel.c_str(), &gps_handler, NULL);

    // control-c handler
    signal(SIGINT,sighandler);

    printf("Receiving LCM:\n\tAirspeed:%s\n\tGPS: %s\n\tAltitude: %s\n\nSending LCM:\n\t%s\n", airspeed_in_channel.c_str(), gps_channel.c_str(), altimeter_channel.c_str(), airspeed_out_channel.c_str());

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


    mav_indexed_measurement_t_unsubscribe(lcm_, airspeed_in_sub);
    mav_indexed_measurement_t_unsubscribe(lcm_, altitude_sub);
    mav_gps_data_t_unsubscribe(lcm_, gps_sub);

    lcm_destroy (lcm_);

    if (rolling_stats != NULL) {
        delete rolling_stats;
    }

    printf("done.\n");

    exit(0);
}
