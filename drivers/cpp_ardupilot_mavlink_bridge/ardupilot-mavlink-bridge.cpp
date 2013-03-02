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
 *      #include "../mavlink-generated2/ardupilotmega/mavlink.h"
 * to 
 *      ../../LCM/mavlink_message_t.h
 * and comment out its definition of
 *  mavlink_message to make this work
 * 
 * XXX XXX XXX XXX
 */


lcm_t * lcmGps;
lcm_t * lcmAttitude;
lcm_t * lcmBaroAirspeed;
lcm_t * lcm;

char *channelAttitude = NULL;
char *channelBaroAirspeed = NULL;
char *channelGps = NULL;

mavlink_msg_container_t_subscription_t * mavlink_sub;

static void usage(void)
{
        fprintf(stderr, "usage: ardupilot-mavlink-bridge mavlink-channel-name attitude-channel-name baro-airspeed-channel-name gps-channel-name\n");
        fprintf(stderr, "    mavlink-channel-name : LCM channel name with MAVLINK LCM messages\n");
        fprintf(stderr, "    attitude-channel-name : LCM channel to publish attitude messages on\n");
        fprintf(stderr, "    baro-airspeed-channel-name : LCM channel to publish barometric altitude and airspeed\n");
        fprintf(stderr, "    gps-channel-name : LCM channel to publish GPS messages on\n");
        fprintf(stderr, "  example:\n");
        fprintf(stderr, "    ardupilot-mavlink-bridge MAVLINK attitude baro-airspeed gps\n");
        fprintf(stderr, "    reads LCM MAVLINK messages and converts them to easy to use attitude, baro/airspeed and gps messages\n");
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
            lcmt_attitude_publish (lcmAttitude, channelAttitude, &attitudeMsg);
            break;
         /*   
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
            
            lcmt_gps_publish (lcmGps, channelGps, &gpsMsg);
            
            
            break;
           */ 
        case MAVLINK_MSG_ID_GPS_RAW_INT:
            mavlink_gps_raw_int_t pos;
            mavlink_msg_gps_raw_int_decode(&mavmsg, &pos);
            
            // convert to LCM type
            lcmt_gps gpsMsg;
            gpsMsg.timestamp = getTimestampNow();
            
            gpsMsg.fix_type = pos.fix_type;  //0-1: no fix, 2: 2D fix, 3: 3D fix.
            
            gpsMsg.latitude = pos.lat; //Latitude in 1E7 degrees
            gpsMsg.longitude = pos.lon; //Latitude in 1E7 degrees
            gpsMsg.alt = pos.alt; //Altitude in 1E3 meters (millimeters) above MSL
            
            gpsMsg.hdop = pos.eph; //GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
            gpsMsg.vdop = pos.epv; //GPS VDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
            
            gpsMsg.velocity = pos.vel; // GPS ground speed (m/s * 100). If unknown, set to: 65535
            gpsMsg.course_over_ground = pos.cog; //Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535
            gpsMsg.satellites_visible = pos.satellites_visible; //Number of satellites visible. If unknown, set to 255
            
            lcmt_gps_publish (lcmGps, channelGps, &gpsMsg);
            
            
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
            
            lcmt_baro_airspeed_publish (lcmBaroAirspeed, channelBaroAirspeed, &baroAirMsg);
            break;
        default:
            cout << "unknown message id = " << mavmsg.msgid << endl;
            break;
            
    }
}


int main(int argc,char** argv)
{
    char *channelMavlink = NULL;

    if (argc!=5) {
        usage();
        exit(0);
    }

    channelMavlink = argv[1];
    channelAttitude = argv[2];
    channelBaroAirspeed = argv[3];
    channelGps = argv[4];

    lcm = lcm_create ("udpm://239.255.76.67:7667?ttl=1");
    if (!lcm)
    {
        fprintf(stderr, "lcm_create for recieve failed.  Quitting.\n");
        return 1;
    }


    lcmGps = lcm_create("udpm://239.255.76.67:7667?ttl=1");
    if (!lcmGps)
    {
        fprintf(stderr, "lcm_create for lcmGps failed.  Quitting.\n");
        return 1;
    }
    
    lcmAttitude = lcm_create("udpm://239.255.76.67:7667?ttl=1");
    if (!lcmAttitude)
    {
        fprintf(stderr, "lcm_create for lcmAttitude failed.  Quitting.\n");
        return 1;
    }
    
    lcmBaroAirspeed = lcm_create("udpm://239.255.76.67:7667?ttl=1");
    if (!lcmBaroAirspeed)
    {
        fprintf(stderr, "lcm_create for lcmBaroAirspeed failed.  Quitting.\n");
        return 1;
    }

    mavlink_sub =  mavlink_msg_container_t_subscribe (lcm, channelMavlink, &mavlink_handler, NULL);


    signal(SIGINT,sighandler);

    printf("Receiving:\n\tMavlink LCM: %s\nPublishing LCM:\n\tAttiude: %s\n\tBarometric altitude and airspeed: %s\n\tGPS: %s\n", channelMavlink, channelAttitude, channelBaroAirspeed, channelGps);

    while (true)
    {
        // read the LCM channel
        lcm_handle (lcm);
    }

    return 0;
}
