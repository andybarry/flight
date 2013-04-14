/*
 * Bridges MAVLINK LCM messages and libbot LCM messages, using the ardupilot
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
#include "../../LCM/lcmt_battery_status.h"
#include "../../LCM/lcmt_attitude.h"
#include "../../LCM/lcmt_baro_airspeed.h"
#include "../../LCM/lcmt_wingeron_u.h"


#include <bot_core/bot_core.h>
#include <bot_param/param_client.h>

#include "mav_ins_t.h" // from Fixie
#include "mav_gps_data_t.h" // from Fixie

#include "mavconn.h" // from mavconn
    
//#include "../../mavlink-generated/ardupilotmega/mavlink.h"
#include "../../mavlink-generated2/csailrlg/mavlink.h"
//#include "../../mavlink-generated2/csailrlg/mavlink_msg_scaled_pressure_and_airspeed.h"

#define GRAVITY_MSS 9.80665f // this matches the ArduPilot definition

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

int origin_init = 0;
BotGPSLinearize gpsLinearize;
double elev_origin;

char *channelAttitude = NULL;
char *channelBaroAirspeed = NULL;
char *channelGps = NULL;
char *channelBatteryStatus = NULL;

mavlink_msg_container_t_subscription_t * mavlink_sub;
lcmt_wingeron_u_subscription_t *wingeron_u_sub;

uint8_t systemID = getSystemID();

static void usage(void)
{
        fprintf(stderr, "usage: ardupilot-mavlink-bridge mavlink-channel-name attitude-channel-name baro-airspeed-channel-name gps-channel-name battery-status-channel-name input-servo-channel-name\n");
        fprintf(stderr, "    mavlink-channel-name : LCM channel name with MAVLINK LCM messages\n");
        fprintf(stderr, "    attitude-channel-name : LCM channel to publish attitude messages on\n");
        fprintf(stderr, "    baro-airspeed-channel-name : LCM channel to publish barometric altitude and airspeed\n");
        fprintf(stderr, "    gps-channel-name : LCM channel to publish GPS messages on\n");
        fprintf(stderr, "    battery-status-channel-name : LCM channel to publish battery status messages on\n");
        fprintf(stderr, "    input-servo-channel-name : LCM channel to listen for servo commands on\n");
        fprintf(stderr, "  example:\n");
        fprintf(stderr, "    ./ardupilot-mavlink-bridge MAVLINK attitude baro-airspeed gps battery-status wingeron_u\n");
        fprintf(stderr, "    reads LCM MAVLINK messages and converts them to easy to use attitude, baro/airspeed and gps messages\n");
}


void sighandler(int dum)
{
    printf("\nClosing... ");

    mavlink_msg_container_t_unsubscribe(lcm, mavlink_sub);
    lcmt_wingeron_u_unsubscribe(lcm, wingeron_u_sub);
    lcm_destroy (lcm);

    printf("done.\n");
    
    exit(0);
}

int64_t getTimestampNow()
{
    struct timeval thisTime;
    gettimeofday(&thisTime, NULL);
    return (thisTime.tv_sec * 1000000.0) + (float)thisTime.tv_usec + 0.5;
}

int EightBitToServoCmd(int charInputIn)
{
    return 200/51 * charInputIn + 1000;
}

void wingeron_u_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_wingeron_u *msg, void *user)
{
    // translate a local LCM message for servos into a message for the airplane over
    // mavlink
    
    int aileronLeft = EightBitToServoCmd(msg->aileronLeft);
    int aileronRight = EightBitToServoCmd(msg->aileronRight);
    int elevator = EightBitToServoCmd(msg->elevator);
    int rudder = EightBitToServoCmd(msg->rudder);
    int throttleFront = EightBitToServoCmd(msg->throttleFront);
    int throttleRear = EightBitToServoCmd(msg->throttleRear);
    
    mavlink_message_t mavmsg;
    
	mavlink_msg_rc_channels_override_pack(systemID, 200, &mavmsg, 1, 200, aileronLeft, elevator, throttleFront, rudder, aileronRight, throttleRear, 1000, 1000);
	// Publish the message on the LCM IPC bus
	sendMAVLinkMessage(lcm, &mavmsg);
    
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
            mavlink_raw_imu_t rawImu;
            mavlink_msg_raw_imu_decode(&mavmsg, &rawImu);
            
            // convert to LCM type
            mav_ins_t insMsg;
            insMsg.utime = getTimestampNow();
            insMsg.device_time = rawImu.time_usec;
            
            insMsg.gyro[0] = (double)rawImu.xgyro/1000;
            insMsg.gyro[1] = (double)rawImu.ygyro/1000;
            insMsg.gyro[2] = (double)rawImu.zgyro/1000;
            
            insMsg.accel[0] = (double)rawImu.xacc/1000*GRAVITY_MSS;
            insMsg.accel[1] = (double)rawImu.yacc/1000*GRAVITY_MSS;
            insMsg.accel[2] = (double)rawImu.zacc/1000*GRAVITY_MSS;
            
            insMsg.mag[0] = rawImu.xmag;
            insMsg.mag[1] = rawImu.ymag;
            insMsg.mag[2] = rawImu.zmag;
            
            insMsg.quat[0] = 0; // unused
            insMsg.quat[1] = 0; // unused
            insMsg.quat[2] = 0; // unused
            insMsg.quat[3] = 0; // unused
            
            insMsg.pressure = 0; // set somewhere else
            insMsg.rel_alt = 0; // set somewhere else
            
            mav_ins_t_publish(lcm, channelAttitude, &insMsg);
            
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
            //DISABLED: USING RAW IMU DATA INSTEAD
            //lcmt_attitude_publish (lcmAttitude, channelAttitude, &attitudeMsg);
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
            mav_gps_data_t gpsMsg;
            gpsMsg.utime = getTimestampNow();
            
            gpsMsg.gps_lock = pos.fix_type;  //0-1: no fix, 2: 2D fix, 3: 3D fix.
            
            gpsMsg.latitude = (double)pos.lat/1e7; //Latitude comes in at 1E7 degrees
            gpsMsg.longitude = (double)pos.lon/1e7; //Latitude comes in at 1E7 degrees
            gpsMsg.elev = (double)pos.alt/1e3; //Altitude comes in at 1E3 meters (millimeters) above MSL
            
            gpsMsg.horizontal_accuracy = pos.eph; //GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
            gpsMsg.vertical_accuracy = pos.epv; //GPS VDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
            
            gpsMsg.speed = (double)pos.vel/100; // GPS ground speed comes in at (m/s * 100). If unknown, set to: 65535
            gpsMsg.heading = (double)pos.cog/100; //Course over ground (NOT heading, but direction of movement) comes in at degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535
            gpsMsg.numSatellites = pos.satellites_visible; //Number of satellites visible. If unknown, set to 255
            
            
            /**
            * Get lineared XYZ.
            * This code from Fixie/drivers/ublox_comm/src/driver/ublox_comm.c
            *
            */
            double latlong[2];
            latlong[0] = gpsMsg.latitude;
            latlong[1] = gpsMsg.longitude;
            double xy[2]; //xy in ENU coordinates
            //require 3d lock to linearize gps
            if (gpsMsg.gps_lock == 3 && origin_init == 1)
            {
                bot_gps_linearize_to_xy(&gpsLinearize, latlong, xy);
            }
            else {
                xy[0] = xy[1] = 0;
            }
            
            double xyz[3];
            xyz[0] = xy[0];
            xyz[1] = xy[1];
            xyz[2] = gpsMsg.elev - elev_origin;
            memcpy(gpsMsg.xyz_pos, xyz, 3 * sizeof(double));
            
            mav_gps_data_t_publish (lcm, channelGps, &gpsMsg);
            
            break;
            
        case MAVLINK_MSG_ID_SCALED_PRESSURE: // hacked this message to give what I want on the firmware side
            mavlink_scaled_pressure_t pressure;
            mavlink_msg_scaled_pressure_decode(&mavmsg, &pressure);
            
            // convert to LCM type
            lcmt_baro_airspeed baroAirMsg;
            baroAirMsg.utime = getTimestampNow();
            
            baroAirMsg.airspeed = pressure.press_abs;   // HACK
            baroAirMsg.baro_altitude = pressure.press_diff + elev_origin;  // HACK
            baroAirMsg.temperature = pressure.temperature;
            
            // airspeed is unreliable under about 1.25 m/s
            if (baroAirMsg.airspeed < 1.5)
            {
             //   baroAirMsg.airspeed = 0;
            }
            lcmt_baro_airspeed_publish (lcm, channelBaroAirspeed, &baroAirMsg);
            break;
            
        case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
            // we sent this message, so ignore it
            break;
            
        case MAVLINK_MSG_ID_BATTERY_STATUS:
            mavlink_battery_status_t batmsg;
            mavlink_msg_battery_status_decode(&mavmsg, &batmsg);
            
            lcmt_battery_status lcmmsg;
            
            lcmmsg.timestamp = getTimestampNow();
            
            lcmmsg.voltage = batmsg.voltage_cell_1/1000.0;
            lcmmsg.amps_now = batmsg.current_battery/100.0;
            lcmmsg.amps_total = batmsg.voltage_cell_6/100.0;
            lcmmsg.percent_remaining = batmsg.battery_remaining;
            
            //cout << "v: " << batmsg.voltage_cell_1/1000.0 << " curr: " << batmsg.current_battery/100.0 << " remain: " << batmsg.battery_remaining <<  " total amph " << batmsg.voltage_cell_6/100.0 << endl;
            
            lcmt_battery_status_publish (lcm, channelBatteryStatus, &lcmmsg);
            break;
            
        case MAVLINK_MSG_ID_STATUSTEXT:
            mavlink_statustext_t textMsg;
            mavlink_msg_statustext_decode(&mavmsg, &textMsg);
            
            
            
            cout << "status text: " << textMsg.text << endl;
            break;
            
        default:
            cout << "unknown message id = " << (int)mavmsg.msgid << endl;
            break;
            
    }
}


int main(int argc,char** argv)
{
    char *channelMavlink = NULL;
    char *channelWingeronU = NULL;

    if (argc!=7) {
        usage();
        exit(0);
    }

    channelMavlink = argv[1];
    channelAttitude = argv[2];
    channelBaroAirspeed = argv[3];
    channelGps = argv[4];
    channelBatteryStatus = argv[5];
    channelWingeronU = argv[6];

    lcm = lcm_create ("udpm://239.255.76.67:7667?ttl=1");
    if (!lcm)
    {
        fprintf(stderr, "lcm_create for recieve failed.  Quitting.\n");
        return 1;
    }

    mavlink_sub =  mavlink_msg_container_t_subscribe (lcm, channelMavlink, &mavlink_handler, NULL);
    wingeron_u_sub = lcmt_wingeron_u_subscribe(lcm, channelWingeronU, &wingeron_u_handler, NULL);

    signal(SIGINT,sighandler);
    
    // init GPS origin
    double latlong_origin[2];
    BotParam *param = bot_param_new_from_server(lcm, 0);
    if (param != NULL) {
        if (bot_param_get_double_array(param, "gps_origin.latlon", latlong_origin, 2) == -1) {
            fprintf(stderr, "error: unable to get gps_origin.latlon from param server\n");
            exit(1); // Don't allow usage without latlon origin.
        } else {
            fprintf(stderr, "\n\nInitializing gps origin at %f,%f\n", latlong_origin[0], latlong_origin[1]);
            bot_gps_linearize_init(&gpsLinearize, latlong_origin);
            origin_init = 1;
        }

        if (bot_param_get_double(param, "gps_origin.elevation", &elev_origin) == -1) {
            fprintf(stderr, "error: unable to get elev_origin from param server, using 0\n");
            elev_origin = 0;
        }
    } else {
        fprintf(stderr, "error: no param server, no gps_origin.latlon\n");
    }

    printf("Receiving:\n\tMavlink LCM: %s\n\tWingeron u: %s\nPublishing LCM:\n\tAttiude: %s\n\tBarometric altitude and airspeed: %s\n\tGPS: %s\n\tBattery status: %s\n", channelMavlink, channelWingeronU, channelAttitude, channelBaroAirspeed, channelGps, channelBatteryStatus);

    while (true)
    {
        // read the LCM channel
        lcm_handle (lcm);
    }

    return 0;
}
