/** @file
 *	@brief MAVLink comm protocol testsuite generated from csailrlg.xml
 *	@see http://qgroundcontrol.org/mavlink/
 */
#ifndef CSAILRLG_TESTSUITE_H
#define CSAILRLG_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_ardupilotmega(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_csailrlg(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_ardupilotmega(system_id, component_id, last_msg);
	mavlink_test_csailrlg(system_id, component_id, last_msg);
}
#endif

#include "../ardupilotmega/testsuite.h"


static void mavlink_test_scaled_pressure_and_airspeed(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_scaled_pressure_and_airspeed_t packet_in = {
		963497464,
	45.0,
	73.0,
	101.0,
	18067,
	};
	mavlink_scaled_pressure_and_airspeed_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.time_boot_ms = packet_in.time_boot_ms;
        	packet1.press_abs = packet_in.press_abs;
        	packet1.press_diff = packet_in.press_diff;
        	packet1.airspeed = packet_in.airspeed;
        	packet1.temperature = packet_in.temperature;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_scaled_pressure_and_airspeed_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_scaled_pressure_and_airspeed_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_scaled_pressure_and_airspeed_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.press_abs , packet1.press_diff , packet1.temperature , packet1.airspeed );
	mavlink_msg_scaled_pressure_and_airspeed_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_scaled_pressure_and_airspeed_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.press_abs , packet1.press_diff , packet1.temperature , packet1.airspeed );
	mavlink_msg_scaled_pressure_and_airspeed_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_scaled_pressure_and_airspeed_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_scaled_pressure_and_airspeed_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.press_abs , packet1.press_diff , packet1.temperature , packet1.airspeed );
	mavlink_msg_scaled_pressure_and_airspeed_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_state_estimator_pose(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_state_estimator_pose_t packet_in = {
		963497464,
	45.0,
	73.0,
	101.0,
	129.0,
	157.0,
	185.0,
	213.0,
	241.0,
	269.0,
	297.0,
	325.0,
	353.0,
	381.0,
	409.0,
	437.0,
	465.0,
	};
	mavlink_state_estimator_pose_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.utime = packet_in.utime;
        	packet1.x = packet_in.x;
        	packet1.y = packet_in.y;
        	packet1.z = packet_in.z;
        	packet1.velx = packet_in.velx;
        	packet1.vely = packet_in.vely;
        	packet1.velz = packet_in.velz;
        	packet1.q1 = packet_in.q1;
        	packet1.q2 = packet_in.q2;
        	packet1.q3 = packet_in.q3;
        	packet1.q4 = packet_in.q4;
        	packet1.rotation_rate_x = packet_in.rotation_rate_x;
        	packet1.rotation_rate_y = packet_in.rotation_rate_y;
        	packet1.rotation_rate_z = packet_in.rotation_rate_z;
        	packet1.accelx = packet_in.accelx;
        	packet1.accely = packet_in.accely;
        	packet1.accelz = packet_in.accelz;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_state_estimator_pose_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_state_estimator_pose_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_state_estimator_pose_pack(system_id, component_id, &msg , packet1.utime , packet1.x , packet1.y , packet1.z , packet1.velx , packet1.vely , packet1.velz , packet1.q1 , packet1.q2 , packet1.q3 , packet1.q4 , packet1.rotation_rate_x , packet1.rotation_rate_y , packet1.rotation_rate_z , packet1.accelx , packet1.accely , packet1.accelz );
	mavlink_msg_state_estimator_pose_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_state_estimator_pose_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.utime , packet1.x , packet1.y , packet1.z , packet1.velx , packet1.vely , packet1.velz , packet1.q1 , packet1.q2 , packet1.q3 , packet1.q4 , packet1.rotation_rate_x , packet1.rotation_rate_y , packet1.rotation_rate_z , packet1.accelx , packet1.accely , packet1.accelz );
	mavlink_msg_state_estimator_pose_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_state_estimator_pose_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_state_estimator_pose_send(MAVLINK_COMM_1 , packet1.utime , packet1.x , packet1.y , packet1.z , packet1.velx , packet1.vely , packet1.velz , packet1.q1 , packet1.q2 , packet1.q3 , packet1.q4 , packet1.rotation_rate_x , packet1.rotation_rate_y , packet1.rotation_rate_z , packet1.accelx , packet1.accely , packet1.accelz );
	mavlink_msg_state_estimator_pose_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_lcm_transport(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_lcm_transport_t packet_in = {
		93372036854775807ULL,
	963497880,
	963498088,
	963498296,
	18275,
	"WXYZABCDEFGHIJKLMNOP",
	"RSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZ",
	};
	mavlink_lcm_transport_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.utime = packet_in.utime;
        	packet1.message_part_counter = packet_in.message_part_counter;
        	packet1.message_part_total = packet_in.message_part_total;
        	packet1.payload_size = packet_in.payload_size;
        	packet1.msg_id = packet_in.msg_id;
        
        	mav_array_memcpy(packet1.channel_name, packet_in.channel_name, sizeof(char)*21);
        	mav_array_memcpy(packet1.payload, packet_in.payload, sizeof(char)*62);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_lcm_transport_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_lcm_transport_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_lcm_transport_pack(system_id, component_id, &msg , packet1.utime , packet1.channel_name , packet1.msg_id , packet1.message_part_counter , packet1.message_part_total , packet1.payload_size , packet1.payload );
	mavlink_msg_lcm_transport_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_lcm_transport_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.utime , packet1.channel_name , packet1.msg_id , packet1.message_part_counter , packet1.message_part_total , packet1.payload_size , packet1.payload );
	mavlink_msg_lcm_transport_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_lcm_transport_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_lcm_transport_send(MAVLINK_COMM_1 , packet1.utime , packet1.channel_name , packet1.msg_id , packet1.message_part_counter , packet1.message_part_total , packet1.payload_size , packet1.payload );
	mavlink_msg_lcm_transport_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_csailrlg(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_scaled_pressure_and_airspeed(system_id, component_id, last_msg);
	mavlink_test_state_estimator_pose(system_id, component_id, last_msg);
	mavlink_test_lcm_transport(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // CSAILRLG_TESTSUITE_H
