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

static void mavlink_test_csailrlg(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_scaled_pressure_and_airspeed(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // CSAILRLG_TESTSUITE_H
