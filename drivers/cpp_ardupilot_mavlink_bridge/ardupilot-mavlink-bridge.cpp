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
#include "../../LCM/lcmt_deltawing_u.h"
#include "../../LCM/lcmt_stereo_control.h"
#include "../../LCM/lcmt_beep.h"


#include <bot_core/bot_core.h>
#include <bot_param/param_client.h>

#include "lcmtypes/mav_ins_t.h" // from pronto
#include "lcmtypes/mav_gps_data_t.h" // from pronto
#include "lcmtypes/mav_altimeter_t.h" // from pronto
#include "lcmtypes/mav_airspeed_t.h" // from pronto

#include "../../externals/ConciseArgs.hpp"

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

#define IGNORE_SYS_ID1 99 // helen's fpga
#define IGNORE_SYS_ID2 42 // this computer


lcm_t * lcm;

int origin_init = 0;
BotGPSLinearize gpsLinearize;
double elev_origin;

int global_beep = 0;

string mavlink_channel = "MAVLINK";
string attitude_channel = "attitude";
string airspeed_channel = "airspeed";
string altimeter_channel = "altimeter";
string gps_channel = "gps";
string battery_status_channel = "battery-status";
string deltawing_u_channel = "deltawing_u";
string servo_out_channel = "servo_out";
string stereo_control_channel = "stereo-control";
string beep_channel = "beep";


mavlink_msg_container_t_subscription_t * mavlink_sub;
lcmt_deltawing_u_subscription_t *deltawing_u_sub;
lcmt_beep_subscription_t *beep_sub;

int last_stereo_control = 0;

uint8_t systemID = getSystemID();

void sighandler(int dum)
{
    printf("\nClosing... ");

    mavlink_msg_container_t_unsubscribe(lcm, mavlink_sub);
    lcmt_deltawing_u_unsubscribe(lcm, deltawing_u_sub);
    lcmt_beep_unsubscribe(lcm, beep_sub);
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

void beep_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_beep *msg, void *user)
{
    global_beep = msg->beep;
}

void deltawing_u_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_deltawing_u *msg, void *user)
{
    // translate a local LCM message for servos into a message for the airplane over
    // mavlink

    int elevonL = EightBitToServoCmd(msg->elevonL);
    int elevonR = EightBitToServoCmd(msg->elevonR);
    int throttle = EightBitToServoCmd(msg->throttle);

    mavlink_message_t mavmsg;

    int beep;
    if (global_beep > 0) {
        beep = 1999;
    } else {
        beep = 1000;
    }

	mavlink_msg_rc_channels_override_pack(systemID, 200, &mavmsg, 1, 200, elevonL, elevonR, throttle, 1000, 1000, 1000, 1000, beep);
	// Publish the message on the LCM IPC bus
	sendMAVLinkMessage(lcm, &mavmsg);

}

void mavlink_handler(const lcm_recv_buf_t *rbuf, const char* channel, const mavlink_msg_container_t *msg, void *user)
{
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

    if (mavmsg.sysid == IGNORE_SYS_ID1 || mavmsg.sysid == IGNORE_SYS_ID2) {
        // this is from a system we are ignoring
        return;
    }

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

            mav_ins_t_publish(lcm, attitude_channel.c_str(), &insMsg);

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
            //gpsMsg.utime = getTimestampNow();
            gpsMsg.utime = pos.time_usec;

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

            mav_gps_data_t_publish (lcm, gps_channel.c_str(), &gpsMsg);

            break;

        case MAVLINK_MSG_ID_SCALED_PRESSURE: // hacked this message to give what I want on the firmware side
            mavlink_scaled_pressure_t pressure;
            mavlink_msg_scaled_pressure_decode(&mavmsg, &pressure);

            // convert to LCM type
            mav_altimeter_t altimeter_msg;
            mav_airspeed_t airspeed_msg;

            int64_t msg_timestamp;
            msg_timestamp = getTimestampNow();

            altimeter_msg.utime = msg_timestamp;
            airspeed_msg.utime = msg_timestamp;

            airspeed_msg.airspeed = pressure.press_abs;   // HACK
            altimeter_msg.altitude = pressure.press_diff + elev_origin;  // HACK
            altimeter_msg.temperature = pressure.temperature;

            // airspeed is unreliable under about 1.25 m/s
            if (airspeed_msg.airspeed < 1.5)
            {
             //   airspeed_msg.airspeed = 0;
            }

            mav_altimeter_t_publish(lcm, altimeter_channel.c_str(), &altimeter_msg);
            mav_airspeed_t_publish(lcm, airspeed_channel.c_str(), &airspeed_msg);
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
            lcmmsg.milliamp_hours_total = batmsg.voltage_cell_6/100.0;
            lcmmsg.percent_remaining = batmsg.battery_remaining;

            //cout << "v: " << batmsg.voltage_cell_1/1000.0 << " curr: " << batmsg.current_battery/100.0 << " remain: " << batmsg.battery_remaining <<  " total amph " << batmsg.voltage_cell_6/100.0 << endl;

            lcmt_battery_status_publish (lcm, battery_status_channel.c_str(), &lcmmsg);
            break;

        case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:

            // decode the mavlink message

            mavlink_servo_output_raw_t servomsg;
            mavlink_msg_servo_output_raw_decode(&mavmsg, &servomsg);

            // create the LCM message
            lcmt_deltawing_u servoOutMsg;

            servoOutMsg.timestamp = getTimestampNow();

            /*
             * Output channels:
             *  1: Elevon L
             *  2: Elevon R
             *  3: Throttle
             *  4:
             *  5: autonmous switch
             *  6: video record switch
             *  7:
             *  8:
             */

            servoOutMsg.elevonL = servomsg.servo1_raw;
            servoOutMsg.elevonR = servomsg.servo2_raw;
            servoOutMsg.throttle = servomsg.servo3_raw;

            if (servomsg.servo5_raw > 1500)
            {
                servoOutMsg.is_autonomous = 1;
            } else {
                servoOutMsg.is_autonomous = 0;
            }

            if (servomsg.servo6_raw > 1500)
            {
                servoOutMsg.video_record = 1;
            } else {
                servoOutMsg.video_record = 0;
            }

            if (last_stereo_control != servoOutMsg.video_record)
            {
                // something has changed, send a new message
                lcmt_stereo_control stereo_control_msg;
                stereo_control_msg.timestamp = getTimestampNow();
                stereo_control_msg.stereo_control =
                     servoOutMsg.video_record;

                lcmt_stereo_control_publish(lcm, stereo_control_channel.c_str(),
                    &stereo_control_msg);

                last_stereo_control = servoOutMsg.video_record;
            }

            // send the lcm message
            lcmt_deltawing_u_publish(lcm, servo_out_channel.c_str(), &servoOutMsg);


            break;

        case MAVLINK_MSG_ID_STATUSTEXT:
            mavlink_statustext_t textMsg;
            mavlink_msg_statustext_decode(&mavmsg, &textMsg);



            cout << "status text: " << textMsg.text << endl;
            break;

        default:
            cout << "unknown message id = " << (int)mavmsg.msgid << " from sysid = " << (float)mavmsg.sysid << endl;
            break;

    }
}


int main(int argc,char** argv)
{

    ConciseArgs parser(argc, argv);
    parser.add(mavlink_channel, "m", "mavlink-channel" "LCM channel for mavlink.");
    parser.add(attitude_channel, "a", "attitude-channel" "LCM channel for IMU messages.");
    parser.add(airspeed_channel, "s", "airspeed-channel" "LCM channel for pitot tube messages.");
    parser.add(altimeter_channel, "h", "altimeter-channel" "LCM channel for the altimeter.");
    parser.add(gps_channel, "g", "gps-channel" "LCM channel for the GPS.");
    parser.add(battery_status_channel, "b", "batter-status-channel" "LCM channel for battery status.");
    parser.add(deltawing_u_channel, "d", "deltawing-u-channel" "LCM channel for deltawing control messages.");
    parser.add(servo_out_channel, "v", "servo-out-channel" "LCM channel for servo commands sent by the aircraft.");
    parser.add(stereo_control_channel, "c", "stereo-control-channel" "LCM channel for stereo control.");
    parser.add(beep_channel, "b", "beep-channel" "LCM channel for beep messages.");
    parser.parse();


    lcm = lcm_create ("udpm://239.255.76.67:7667?ttl=1");
    if (!lcm)
    {
        fprintf(stderr, "lcm_create for recieve failed.  Quitting.\n");
        return 1;
    }

    mavlink_sub =  mavlink_msg_container_t_subscribe (lcm, mavlink_channel.c_str(), &mavlink_handler, NULL);
    deltawing_u_sub = lcmt_deltawing_u_subscribe(lcm, deltawing_u_channel.c_str(), &deltawing_u_handler, NULL);
    beep_sub = lcmt_beep_subscribe(lcm, beep_channel.c_str(), &beep_handler, NULL);

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

    printf("Receiving:\n\tMavlink LCM: %s\n\tDeltawing u: %s\n\tBeep: %s\nPublishing LCM:\n\tAttiude: %s\n\tBarometric altitude: %s\n\tAirspeed: %s\n\tGPS: %s\n\tBattery status: %s\n\tServo Outputs: %s\n\tStereo Control: %s\n", mavlink_channel.c_str(), deltawing_u_channel.c_str(), beep_channel.c_str(), attitude_channel.c_str(), altimeter_channel.c_str(), airspeed_channel.c_str(), gps_channel.c_str(), battery_status_channel.c_str(), servo_out_channel.c_str(), stereo_control_channel.c_str());

    while (true)
    {
        // read the LCM channel
        lcm_handle (lcm);
    }

    return 0;
}
