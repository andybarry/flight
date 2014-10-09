/*
 * Bridges MAVLINK LCM messages for Helen's FPGA.
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


/* XXX XXX XXX XXX
 *
 * You must add
 *      #include "../mavlink-generated2/ardupilotmega/mavlink.h"
 * to
 *      ../../LCM/mavlink_message_t.h
 * and comment out its definition of
 *  mavlink_message to make this work
 *
 * XXX XXX XXX XXX
 */


lcm_t * lcm;

char *channelStereoControl = NULL;
char *channelMavlink = NULL;
char *channelServoOut = NULL;


mavlink_msg_container_t_subscription_t * mavlink_sub;
lcmt_stereo_control_subscription_t *stereo_control_sub;
lcmt_deltawing_u_subscription_t *servo_out_sub;

uint8_t systemID = getSystemID();

static void usage(void) {
        fprintf(stderr, "usage: fpga-mavlink-bridge stereo-control-channel-name mavlink-channel-name\n");
        fprintf(stderr, "    stereo-control-channel-name : LCM channel to receive stereo control commands on\n");
        fprintf(stderr, "    mavlink-channel-name : LCM channel name with MAVLINK LCM messages\n");
        fprintf(stderr, "    servo-out-channel-name : LCM channel name with servo out messages on it for sending timestamps\n");
        fprintf(stderr, "  example:\n");
        fprintf(stderr, "    ./fpga-mavlink-bridge stereo-control MAVLINK servo_out\n");
}


void sighandler(int dum) {
    printf("\nClosing... ");

    lcmt_stereo_control_unsubscribe(lcm, stereo_control_sub);
    lcm_destroy (lcm);

    printf("done.\n");

    exit(0);
}

int64_t getTimestampNow() {
    struct timeval thisTime;
    gettimeofday(&thisTime, NULL);
    return (thisTime.tv_sec * 1000000.0) + (float)thisTime.tv_usec + 0.5;
}
void stereo_control_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_stereo_control *msg, void *user) {

    mavlink_message_t mavmsg;

	mavlink_msg_param_set_pack(systemID, 200, &mavmsg, FPGA_TARGET_SYSTEM_ID, FPGA_TARGET_COMPONENT_ID, "RECORD_IMG", (float) msg->stereo_control, MAVLINK_TYPE_FLOAT);

    cout << "sending set param message from " << systemID << " with value " << (float) msg->stereo_control << endl;

	// Publish the message on the LCM IPC bus
	sendMAVLinkMessage(lcm, &mavmsg);

}

void servo_out_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_deltawing_u *msg, void *user) {
    // send a timestamp message on every servo_out message

    mavlink_message_t mavmsg;

	mavlink_msg_system_time_pack(systemID, 200, &mavmsg, getTimestampNow(), 0);

    //cout << "sending timestamp" << endl;

	// Publish the message on the LCM IPC bus
	sendMAVLinkMessage(lcm, &mavmsg);

}

void mavlink_handler(const lcm_recv_buf_t *rbuf, const char* channel, const mavlink_msg_container_t *msg, void *user) {
    // load MAVLINK LCM message

    // debug -- print message
    /*
    lcmt_deltawing_gains msg2 = ConvertFromMidiLcmToPlane(msg);

	struct timeval thisTime;
	gettimeofday(&thisTime, NULL);
    msg2.timestamp = (thisTime.tv_sec * 1000.0) + (float)thisTime.tv_usec/1000.0 + 0.5;

    lastMsg = msg2;

	// send via LCM
	lcmt_deltawing_gains_publish (lcmSend, lcm_out, &msg2);
*/
    // extract the message out of the container
    mavlink_message_t mavmsg = msg->msg;


    if (mavmsg.sysid != FPGA_TARGET_SYSTEM_ID) {
        return;
    }

    switch(mavmsg.msgid) {
        // process messages here
        case MAVLINK_MSG_ID_STATUSTEXT:
            mavlink_statustext_t textMsg;
            mavlink_msg_statustext_decode(&mavmsg, &textMsg);

            cout << "status text: " << textMsg.text << endl;
            break;

        case MAVLINK_MSG_ID_HEARTBEAT:
            cout << "heatbeat from system: " << (int) mavmsg.sysid << endl;
            break;

        default:
            cout << "unknown message id = " << (int)mavmsg.msgid << endl;
            break;

    }
}


int main(int argc,char** argv) {

    if (argc!=4) {
        usage();
        exit(0);
    }

    channelStereoControl = argv[1];
    channelMavlink = argv[2];
    channelServoOut = argv[3];

    lcm = lcm_create ("udpm://239.255.76.67:7667?ttl=1");
    if (!lcm) {
        fprintf(stderr, "lcm_create for recieve failed.  Quitting.\n");
        return 1;
    }

    mavlink_sub =  mavlink_msg_container_t_subscribe (lcm, channelMavlink, &mavlink_handler, NULL);
    stereo_control_sub = lcmt_stereo_control_subscribe(lcm, channelStereoControl, &stereo_control_handler, NULL);
    servo_out_sub = lcmt_deltawing_u_subscribe(lcm, channelServoOut, &servo_out_handler, NULL);

    signal(SIGINT,sighandler);

    printf("Receiving:\n\tStereo Control: %s\n\tMAVLINK: %s\n\tServo out: %s\n", channelStereoControl, channelMavlink, channelServoOut);

    while (true) {
        // read the LCM channel
        lcm_handle (lcm);
    }

    return 0;
}
