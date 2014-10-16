/*
 * Sends a signal to the FPGA to reset the cameras.
 *
 * Author: Andrew Barry, <abarry@csail.mit.edu> 2014
 *
 */


#include <iostream>


using namespace std;


#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>

#include "../../LCM/mavlink_msg_container_t.h"
#include "../../LCM/lcmt_stereo_control.h"
#include "../../LCM/lcmt_deltawing_u.h"


#include <bot_core/bot_core.h>
#include <bot_param/param_client.h>

#include "mav_ins_t.h" // from Fixie
#include "mav_gps_data_t.h" // from Fixie

#include "mavconn.h" // from mavconn

//#include "../../mavlink-generated/ardupilotmega/mavlink.h"
#include "../../mavlink-generated2/csailrlg/mavlink.h"
//#include "../../mavlink-generated2/csailrlg/mavlink_msg_scaled_pressure_and_airspeed.h"

#define FPGA_TARGET_SYSTEM_ID 99
#define FPGA_TARGET_COMPONENT_ID 48

#define RESET_MSG_VALUE 1

using namespace std;

int64_t getTimestampNow()
{
    struct timeval thisTime;
    gettimeofday(&thisTime, NULL);
    return (thisTime.tv_sec * 1000000.0) + (float)thisTime.tv_usec + 0.5;
}

int main(int argc,char** argv) {

    lcm_t * lcm;

    uint8_t systemID = getSystemID();

    lcm = lcm_create ("udpm://239.255.76.67:7667?ttl=1");
    if (!lcm) {
        fprintf(stderr, "lcm_create for recieve failed.  Quitting.\n");
        return 1;
    }

    mavlink_message_t mavmsg;

	mavlink_msg_param_set_pack(systemID, 200, &mavmsg, FPGA_TARGET_SYSTEM_ID, FPGA_TARGET_COMPONENT_ID, "RESET_CAMS", RESET_MSG_VALUE, MAVLINK_TYPE_FLOAT);

    cout << "sending reset signal to cameras from " << (float)systemID << " with value " << RESET_MSG_VALUE << "..." << endl;

	// Publish the message on the LCM IPC bus
	sendMAVLinkMessage(lcm, &mavmsg);

    cout << "done." << endl;

    return 0;
}

