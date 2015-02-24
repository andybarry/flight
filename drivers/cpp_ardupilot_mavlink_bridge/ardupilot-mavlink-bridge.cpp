/*
 * Bridges MAVLINK LCM messages and libbot LCM messages, using the ardupilot
 *
 * Author: Andrew Barry, <abarry@csail.mit.edu> 2013-2015
 *
 */

#include "ardupilot-mavlink-bridge.hpp"

#define IGNORE_SYS_ID1 99 // helen's fpga
#define IGNORE_SYS_ID2 42 // this computer


lcm_t * lcm_;

int origin_init = 0;
BotGPSLinearize gpsLinearize;
double elev_origin;

int global_beep = 0;

std::string mavlink_channel = "MAVLINK";
std::string attitude_channel = "attitude";
std::string airspeed_channel = "airspeed";
std::string altimeter_channel = "altimeter";
std::string sideslip_channel = "sideslip";
std::string gps_channel = "gps";
std::string battery_status_channel = "battery-status";
std::string deltawing_u_channel = "deltawing_u";
std::string servo_out_channel = "servo_out";
std::string stereo_control_channel = "stereo-control";
std::string beep_channel = "beep";
std::string tvlqr_control_channel = "tvlqr-action";


mavlink_msg_container_t_subscription_t * mavlink_sub;
lcmt_deltawing_u_subscription_t *deltawing_u_sub;
lcmt_beep_subscription_t *beep_sub;

int last_stereo_control = 0;
int last_traj_switch = -1;

double altimeter_r, airspeed_r, sideslip_r;

uint8_t systemID = getSystemID();

void sighandler(int dum)
{
    printf("\nClosing... ");

    mavlink_msg_container_t_unsubscribe(lcm_, mavlink_sub);
    lcmt_deltawing_u_unsubscribe(lcm_, deltawing_u_sub);
    lcmt_beep_unsubscribe(lcm_, beep_sub);
    lcm_destroy (lcm_);

    printf("done.\n");

    exit(0);
}

int64_t getTimestampNow()
{
    struct timeval thisTime;
    gettimeofday(&thisTime, NULL);
    return (thisTime.tv_sec * 1000000.0) + (float)thisTime.tv_usec + 0.5;
}

void beep_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_beep *msg, void *user)
{
    global_beep = msg->beep;
}

void deltawing_u_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_deltawing_u *msg, void *user)
{
    // translate a local LCM message for servos into a message for the airplane over
    // mavlink

    mavlink_message_t mavmsg;

    int beep;
    if (global_beep > 0) {
        beep = 1999;
    } else {
        beep = 1000;
    }

    mavlink_msg_rc_channels_override_pack(systemID, 200, &mavmsg, 1, 200, msg->elevonL, msg->elevonR, msg->throttle, 1000, 1000, 1000, 1000, beep);
    // Publish the message on the LCM IPC bus
    sendMAVLinkMessage(lcm_, &mavmsg);

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

    Eigen::Vector3i state_estimator_index;

    switch(mavmsg.msgid)
    {
        // process messages here
        case MAVLINK_MSG_ID_HEARTBEAT:
            std::cout << "got a heartbeat" << std::endl;
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

            mav_ins_t_publish(lcm_, attitude_channel.c_str(), &insMsg);

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

            mav_gps_data_t_publish (lcm_, gps_channel.c_str(), &gpsMsg);

            break;

        case MAVLINK_MSG_ID_SCALED_PRESSURE: // hacked this message to give what I want on the firmware side
            mavlink_scaled_pressure_t pressure;
            mavlink_msg_scaled_pressure_decode(&mavmsg, &pressure);

            // convert to LCM type
            mav_indexed_measurement_t altimeter_msg;
            mav_indexed_measurement_t airspeed_msg;
            mav_indexed_measurement_t sideslip_msg;

            int64_t msg_timestamp;
            msg_timestamp = getTimestampNow();


            // altimeter
            altimeter_msg.utime = msg_timestamp;
            altimeter_msg.state_utime = msg_timestamp;

            double altitude;
            altitude = pressure.press_diff + elev_origin;  // HACK

            altimeter_msg.measured_dim = 1; // altitude only measures 1 dimension (z-axis)

            int altimeter_z_ind[1];

            state_estimator_index = eigen_utils::RigidBodyState::positionInds();

            altimeter_z_ind[0] = state_estimator_index[2]; // measurement on the Z axis (index = 2)
            altimeter_msg.z_indices = altimeter_z_ind;

            double altimeter_value[1];
            altimeter_value[0] = altitude;
            altimeter_msg.z_effective = altimeter_value;

            altimeter_msg.measured_cov_dim = 1;

            double altimeter_cov[1];
            altimeter_cov[0] = altimeter_r;
            altimeter_msg.R_effective = altimeter_cov;

            //altimeter_msg.temperature = pressure.temperature;



            airspeed_msg.utime = msg_timestamp;
            airspeed_msg.state_utime = msg_timestamp;

            double airspeed;
            airspeed = pressure.press_abs;  // HACK

            airspeed_msg.measured_dim = 1; // altitude only measures 1 dimension (x-axis)

            int airspeed_z_ind[1];
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





            sideslip_msg.utime = msg_timestamp;
            sideslip_msg.state_utime = msg_timestamp;

            double sideslip;
            sideslip = 0;

            sideslip_msg.measured_dim = 1; // altitude only measures 1 dimension (y-axis)

            int sideslip_z_ind[1];
            state_estimator_index = eigen_utils::RigidBodyState::velocityInds();
            sideslip_z_ind[0] = state_estimator_index[1]; // measurement on the Y axis (index = 1)
            sideslip_msg.z_indices = sideslip_z_ind;

            double sideslip_value[1];
            sideslip_value[0] = sideslip;
            sideslip_msg.z_effective = sideslip_value;

            sideslip_msg.measured_cov_dim = 1;

            double sideslip_cov[1];
            sideslip_cov[0] = sideslip_r;
            sideslip_msg.R_effective = sideslip_cov;



            mav_indexed_measurement_t_publish(lcm_, altimeter_channel.c_str(), &altimeter_msg);
            mav_indexed_measurement_t_publish(lcm_, airspeed_channel.c_str(), &airspeed_msg);
            mav_indexed_measurement_t_publish(lcm_, sideslip_channel.c_str(), &sideslip_msg);

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

            //std::cout << "v: " << batmsg.voltage_cell_1/1000.0 << " curr: " << batmsg.current_battery/100.0 << " remain: " << batmsg.battery_remaining <<  " total amph " << batmsg.voltage_cell_6/100.0 << std::endl;

            lcmt_battery_status_publish (lcm_, battery_status_channel.c_str(), &lcmmsg);
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
             *  6: trajectory selection switch
             *  7:
             *  8:
             */

            servoOutMsg.elevonL = servomsg.servo1_raw;
            servoOutMsg.elevonR = servomsg.servo2_raw;
            servoOutMsg.throttle = servomsg.servo3_raw;

            if (servomsg.servo5_raw > 1500)
            {
                servoOutMsg.is_autonomous = 1;
                servoOutMsg.video_record = 1;
            } else {
                servoOutMsg.is_autonomous = 0;
                servoOutMsg.video_record = 0;
            }

            // send the lcm message
            lcmt_deltawing_u_publish(lcm_, servo_out_channel.c_str(), &servoOutMsg);

            int traj_switch;

            if (servomsg.servo6_raw > 1700) {
                // switch position 0
                traj_switch = 0;
            } else if (servomsg.servo6_raw > 1300) {
                // switch position 1

                traj_switch = 1;
            } else {
                // switch position 2

                traj_switch = 2;
            }

            if (last_stereo_control != servoOutMsg.video_record)
            {
                // something has changed, send a new message
                lcmt_stereo_control stereo_control_msg;
                stereo_control_msg.timestamp = getTimestampNow();
                stereo_control_msg.stereo_control =
                     servoOutMsg.video_record;

                lcmt_stereo_control_publish(lcm_, stereo_control_channel.c_str(),
                    &stereo_control_msg);

                last_stereo_control = servoOutMsg.video_record;


                // send trajectory messages with autonomous flight messages
                if (servoOutMsg.is_autonomous == 1) {
                    lcmt_tvlqr_controller_action traj_msg;

                    traj_msg.timestamp = getTimestampNow();

                    traj_msg.trajectory_number = traj_switch;

                    last_traj_switch = traj_switch;

                    lcmt_tvlqr_controller_action_publish(lcm_, tvlqr_control_channel.c_str(), &traj_msg);
                }
            }




            break;

        case MAVLINK_MSG_ID_STATUSTEXT:
            mavlink_statustext_t textMsg;
            mavlink_msg_statustext_decode(&mavmsg, &textMsg);



            std::cout << "status text: " << textMsg.text << std::endl;
            break;

        default:
            std::cout << "unknown message id = " << (int)mavmsg.msgid << " from sysid = " << (float)mavmsg.sysid << std::endl;
            break;

    }
}


int main(int argc,char** argv)
{

    ConciseArgs parser(argc, argv);
    parser.add(mavlink_channel, "m", "mavlink-channel", "LCM channel for mavlink.");
    parser.add(attitude_channel, "i", "attitude-channel", "LCM channel for IMU messages.");
    parser.add(airspeed_channel, "s", "airspeed-channel", "LCM channel for pitot tube messages.");
    parser.add(sideslip_channel, "l", "sideslip-channel", "LCM channel for sideslip messages.");
    parser.add(altimeter_channel, "a", "altimeter-channel", "LCM channel for the altimeter.");
    parser.add(gps_channel, "g", "gps-channel", "LCM channel for the GPS.");
    parser.add(battery_status_channel, "b", "batter-status-channel", "LCM channel for battery status.");
    parser.add(deltawing_u_channel, "d", "deltawing-u-channel", "LCM channel for deltawing control messages.");
    parser.add(servo_out_channel, "v", "servo-out-channel", "LCM channel for servo commands sent by the aircraft.");
    parser.add(stereo_control_channel, "c", "stereo-control-channel", "LCM channel for stereo control.");
    parser.add(beep_channel, "p", "beep-channel", "LCM channel for beep messages.");
    parser.add(tvlqr_control_channel, "t", "tvlqr-control-channel", "LCM channel for TVLQR control messages.");
    parser.parse();


    lcm_ = lcm_create ("udpm://239.255.76.67:7667?ttl=1");
    if (!lcm_)
    {
        fprintf(stderr, "lcm_create for recieve failed.  Quitting.\n");
        return 1;
    }

    mavlink_sub =  mavlink_msg_container_t_subscribe (lcm_, mavlink_channel.c_str(), &mavlink_handler, NULL);
    deltawing_u_sub = lcmt_deltawing_u_subscribe(lcm_, deltawing_u_channel.c_str(), &deltawing_u_handler, NULL);
    beep_sub = lcmt_beep_subscribe(lcm_, beep_channel.c_str(), &beep_handler, NULL);

    signal(SIGINT,sighandler);

    // init GPS origin
    double latlong_origin[2];
    BotParam *param = bot_param_new_from_server(lcm_, 0);
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

        altimeter_r = bot_param_get_double_or_fail(param, "state_estimator.altimeter.r");
        airspeed_r = bot_param_get_double_or_fail(param, "state_estimator.airspeed.r");
        sideslip_r = bot_param_get_double_or_fail(param, "state_estimator.sideslip.r");




    } else {
        fprintf(stderr, "Rrror: no param server, no gps_origin.latlon\n");
        fprintf(stderr, "Error: no param server, can't find R values for state estimator.\n");
        exit(1);
    }

    printf("Receiving:\n\tMavlink LCM: %s\n\tDeltawing u: %s\n\tBeep: %s\nPublishing LCM:\n\tAttiude: %s\n\tBarometric altitude: %s\n\tAirspeed: %s\n\tGPS: %s\n\tBattery status: %s\n\tServo Outputs: %s\n\tStereo Control: %s\n\tTVLQR Control: %s\n", mavlink_channel.c_str(), deltawing_u_channel.c_str(), beep_channel.c_str(), attitude_channel.c_str(), altimeter_channel.c_str(), airspeed_channel.c_str(), gps_channel.c_str(), battery_status_channel.c_str(), servo_out_channel.c_str(), stereo_control_channel.c_str(), tvlqr_control_channel.c_str());

    while (true)
    {
        // read the LCM channel
        lcm_handle (lcm_);
    }

    return 0;
}
