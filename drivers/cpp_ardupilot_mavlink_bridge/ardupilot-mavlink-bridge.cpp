/*
 * Integrates the 3d stereo data (from LCM) and the IMU data (from mavlink's LCM bridge)
 *
 * Author: Andrew Barry, <abarry@csail.mit.edu> 2013
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
#include "../../LCM/lcmt_gps.h"
#include "../../LCM/lcmt_attitude.h"
#include "../../LCM/lcmt_baro_airspeed.h"
    
//#include "../../mavlink-generated/ardupilotmega/mavlink.h"
#include "../../mavlink-generated2/csailrlg/mavlink.h"
//#include "../../mavlink-generated2/csailrlg/mavlink_msg_scaled_pressure_and_airspeed.h"


/* XXX XXX XXX XXX
 *
 * You must add
 *      #include "../mavlink-generated/ardupilotmega/mavlink.h"
 * to 
 *      ../LCM/mavlink_message_t.h
 * and comment out its definition of
 *  mavlink_message to make this work
 * 
 * XXX XXX XXX XXX
 */


lcm_t * lcmGps;
lcm_t * lcmAttitude;
lcm_t * lcmBaroAirspeed;
lcm_t * lcm;
char *lcm_out = NULL;
mavlink_msg_container_t_subscription_t * mavlink_sub;

static void usage(void)
{
        fprintf(stderr, "usage: cam-imu-integration input-channel-name output-channel-name\n");
        fprintf(stderr, "    input-channel-name : LCM channel name with MAVLINK LCM messages\n");
        fprintf(stderr, "    output-channel-name : LCM channel name with plane messages\n");
        fprintf(stderr, "  example:\n");
        fprintf(stderr, "    cam-imu-integration MAVLINK temp\n");
        fprintf(stderr, "    reads LCM MAVLINK messages and TODO\n");
}


void sighandler(int dum)
{
    printf("\nClosing... ");

    mavlink_msg_container_t_unsubscribe(lcm, mavlink_sub);
    lcm_destroy (lcm);

    printf("done.\n");
    
    exit(0);
}

int64_t getTimestampNow()
{
    struct timeval thisTime;
    gettimeofday(&thisTime, NULL);
    return (thisTime.tv_sec * 1000.0) + (float)thisTime.tv_usec/1000.0 + 0.5;
}

void mavlink_handler(const lcm_recv_buf_t *rbuf, const char* channel, const mavlink_msg_container_t *msg, void *user)
{
    // load MAVLINK LCM message

    // debug -- print message
    /*
    lcmt_wingeron_gains msg2 = ConvertFromMidiLcmToPlane(msg);
	
	struct timeval thisTime;
	gettimeofday(&thisTime, NULL);
    msg2.timestamp = (thisTime.tv_sec * 1000.0) + (float)thisTime.tv_usec/1000.0 + 0.5;
    
    lastMsg = msg2;
	
	// send via LCM
	lcmt_wingeron_gains_publish (lcmSend, lcm_out, &msg2);
*/
    // extract the message out of the container
    mavlink_message_t mavmsg = msg->msg;
    
    

    switch(mavmsg.msgid)
    {
        // process messages here
        case MAVLINK_MSG_ID_HEARTBEAT:
            cout << "got a heartbeat" << endl;
            break;
        case MAVLINK_MSG_ID_RAW_IMU:
            cout << "got an imu message" << endl;
            mavlink_raw_imu_t rawImu;
            mavlink_msg_raw_imu_decode(&mavmsg, &rawImu);
            cout << rawImu.xacc << ", " << rawImu.yacc << endl;
            break;
        case MAVLINK_MSG_ID_ATTITUDE:
            mavlink_attitude_t attitude;
            mavlink_msg_attitude_decode(&mavmsg, &attitude);
            
            // convert to LCM type
            lcmt_attitude attitudeMsg;
            attitudeMsg.timestamp = getTimestampNow();
            
            attitudeMsg.roll = attitude.roll;
            attitudeMsg.pitch = attitude.pitch;
            attitudeMsg.yaw = attitude.yaw;
            
            attitudeMsg.rollspeed = attitude.rollspeed;
            attitudeMsg.pitchspeed = attitude.pitchspeed;
            attitudeMsg.yawspeed = attitude.yawspeed;
            lcmt_attitude_publish (lcmAttitude, "attitude", &attitudeMsg);
            break;
            
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
            mavlink_global_position_int_t pos;
            mavlink_msg_global_position_int_decode(&mavmsg, &pos);
            
            // convert to LCM type
            lcmt_gps gpsMsg;
            gpsMsg.timestamp = getTimestampNow();
            
            gpsMsg.latitude = pos.lat;
            gpsMsg.longitude = pos.lon;
            gpsMsg.alt = pos.alt;
            gpsMsg.relative_alt = pos.relative_alt;
            
            gpsMsg.vx = pos.vx;
            gpsMsg.vy = pos.vy;
            gpsMsg.vz = pos.vz;
            gpsMsg.hdg = pos.hdg;
            
            lcmt_gps_publish (lcmGps, "gps", &gpsMsg);
            
            
            break;
            
        case MAVLINK_MSG_ID_SCALED_PRESSURE: // hacked this message to give what I want on the firmware side
            mavlink_scaled_pressure_t pressure;
            mavlink_msg_scaled_pressure_decode(&mavmsg, &pressure);
            
            // convert to LCM type
            lcmt_baro_airspeed baroAirMsg;
            baroAirMsg.timestamp = getTimestampNow();
            
            baroAirMsg.airspeed = pressure.press_abs;   // HACK
            baroAirMsg.baro_altitude = pressure.press_diff;  // HACK
            baroAirMsg.temperature = pressure.temperature;
            
            lcmt_baro_airspeed_publish (lcmBaroAirspeed, "baro_airspeed", &baroAirMsg);
            break;
        default:
            cout << "unknown message id = " << mavmsg.msgid << endl;
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


    lcmGps = lcm_create("udpm://239.255.76.67:7667?ttl=0");
    if (!lcmGps)
    {
        fprintf(stderr, "lcm_create for lcmGps failed.  Quitting.\n");
        return 1;
    }
    
    lcmAttitude = lcm_create("udpm://239.255.76.67:7667?ttl=0");
    if (!lcmAttitude)
    {
        fprintf(stderr, "lcm_create for lcmAttitude failed.  Quitting.\n");
        return 1;
    }
    
    lcmBaroAirspeed = lcm_create("udpm://239.255.76.67:7667?ttl=0");
    if (!lcmBaroAirspeed)
    {
        fprintf(stderr, "lcm_create for lcmBaroAirspeed failed.  Quitting.\n");
        return 1;
    }

    mavlink_sub =  mavlink_msg_container_t_subscribe (lcm, lcm_in, &mavlink_handler, NULL);



    signal(SIGINT,sighandler);

    printf("Receiving LCM: %s\n", lcm_in);

    while (true)
    {
        // read the LCM channel
        lcm_handle (lcm);
    }

    return 0;
}
