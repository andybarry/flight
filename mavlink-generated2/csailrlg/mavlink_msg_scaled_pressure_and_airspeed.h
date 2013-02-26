// MESSAGE SCALED_PRESSURE_AND_AIRSPEED PACKING

#define MAVLINK_MSG_ID_SCALED_PRESSURE_AND_AIRSPEED 221

typedef struct __mavlink_scaled_pressure_and_airspeed_t
{
 uint32_t time_boot_ms; ///< Timestamp (milliseconds since system boot)
 float press_abs; ///< Absolute pressure (hectopascal)
 float press_diff; ///< Differential pressure 1 (hectopascal)
 float airspeed; ///< Airspeed measurement (meters per second)
 int16_t temperature; ///< Temperature measurement (0.01 degrees celsius)
} mavlink_scaled_pressure_and_airspeed_t;

#define MAVLINK_MSG_ID_SCALED_PRESSURE_AND_AIRSPEED_LEN 18
#define MAVLINK_MSG_ID_221_LEN 18



#define MAVLINK_MESSAGE_INFO_SCALED_PRESSURE_AND_AIRSPEED { \
	"SCALED_PRESSURE_AND_AIRSPEED", \
	5, \
	{  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_scaled_pressure_and_airspeed_t, time_boot_ms) }, \
         { "press_abs", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_scaled_pressure_and_airspeed_t, press_abs) }, \
         { "press_diff", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_scaled_pressure_and_airspeed_t, press_diff) }, \
         { "airspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_scaled_pressure_and_airspeed_t, airspeed) }, \
         { "temperature", NULL, MAVLINK_TYPE_INT16_T, 0, 16, offsetof(mavlink_scaled_pressure_and_airspeed_t, temperature) }, \
         } \
}


/**
 * @brief Pack a scaled_pressure_and_airspeed message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param press_abs Absolute pressure (hectopascal)
 * @param press_diff Differential pressure 1 (hectopascal)
 * @param temperature Temperature measurement (0.01 degrees celsius)
 * @param airspeed Airspeed measurement (meters per second)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_scaled_pressure_and_airspeed_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t time_boot_ms, float press_abs, float press_diff, int16_t temperature, float airspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[18];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_float(buf, 4, press_abs);
	_mav_put_float(buf, 8, press_diff);
	_mav_put_float(buf, 12, airspeed);
	_mav_put_int16_t(buf, 16, temperature);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 18);
#else
	mavlink_scaled_pressure_and_airspeed_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.press_abs = press_abs;
	packet.press_diff = press_diff;
	packet.airspeed = airspeed;
	packet.temperature = temperature;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 18);
#endif

	msg->msgid = MAVLINK_MSG_ID_SCALED_PRESSURE_AND_AIRSPEED;
	return mavlink_finalize_message(msg, system_id, component_id, 18, 63);
}

/**
 * @brief Pack a scaled_pressure_and_airspeed message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param press_abs Absolute pressure (hectopascal)
 * @param press_diff Differential pressure 1 (hectopascal)
 * @param temperature Temperature measurement (0.01 degrees celsius)
 * @param airspeed Airspeed measurement (meters per second)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_scaled_pressure_and_airspeed_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t time_boot_ms,float press_abs,float press_diff,int16_t temperature,float airspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[18];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_float(buf, 4, press_abs);
	_mav_put_float(buf, 8, press_diff);
	_mav_put_float(buf, 12, airspeed);
	_mav_put_int16_t(buf, 16, temperature);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 18);
#else
	mavlink_scaled_pressure_and_airspeed_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.press_abs = press_abs;
	packet.press_diff = press_diff;
	packet.airspeed = airspeed;
	packet.temperature = temperature;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 18);
#endif

	msg->msgid = MAVLINK_MSG_ID_SCALED_PRESSURE_AND_AIRSPEED;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 18, 63);
}

/**
 * @brief Encode a scaled_pressure_and_airspeed struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param scaled_pressure_and_airspeed C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_scaled_pressure_and_airspeed_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_scaled_pressure_and_airspeed_t* scaled_pressure_and_airspeed)
{
	return mavlink_msg_scaled_pressure_and_airspeed_pack(system_id, component_id, msg, scaled_pressure_and_airspeed->time_boot_ms, scaled_pressure_and_airspeed->press_abs, scaled_pressure_and_airspeed->press_diff, scaled_pressure_and_airspeed->temperature, scaled_pressure_and_airspeed->airspeed);
}

/**
 * @brief Send a scaled_pressure_and_airspeed message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param press_abs Absolute pressure (hectopascal)
 * @param press_diff Differential pressure 1 (hectopascal)
 * @param temperature Temperature measurement (0.01 degrees celsius)
 * @param airspeed Airspeed measurement (meters per second)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_scaled_pressure_and_airspeed_send(mavlink_channel_t chan, uint32_t time_boot_ms, float press_abs, float press_diff, int16_t temperature, float airspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[18];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_float(buf, 4, press_abs);
	_mav_put_float(buf, 8, press_diff);
	_mav_put_float(buf, 12, airspeed);
	_mav_put_int16_t(buf, 16, temperature);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SCALED_PRESSURE_AND_AIRSPEED, buf, 18, 63);
#else
	mavlink_scaled_pressure_and_airspeed_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.press_abs = press_abs;
	packet.press_diff = press_diff;
	packet.airspeed = airspeed;
	packet.temperature = temperature;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SCALED_PRESSURE_AND_AIRSPEED, (const char *)&packet, 18, 63);
#endif
}

#endif

// MESSAGE SCALED_PRESSURE_AND_AIRSPEED UNPACKING


/**
 * @brief Get field time_boot_ms from scaled_pressure_and_airspeed message
 *
 * @return Timestamp (milliseconds since system boot)
 */
static inline uint32_t mavlink_msg_scaled_pressure_and_airspeed_get_time_boot_ms(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field press_abs from scaled_pressure_and_airspeed message
 *
 * @return Absolute pressure (hectopascal)
 */
static inline float mavlink_msg_scaled_pressure_and_airspeed_get_press_abs(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field press_diff from scaled_pressure_and_airspeed message
 *
 * @return Differential pressure 1 (hectopascal)
 */
static inline float mavlink_msg_scaled_pressure_and_airspeed_get_press_diff(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field temperature from scaled_pressure_and_airspeed message
 *
 * @return Temperature measurement (0.01 degrees celsius)
 */
static inline int16_t mavlink_msg_scaled_pressure_and_airspeed_get_temperature(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  16);
}

/**
 * @brief Get field airspeed from scaled_pressure_and_airspeed message
 *
 * @return Airspeed measurement (meters per second)
 */
static inline float mavlink_msg_scaled_pressure_and_airspeed_get_airspeed(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Decode a scaled_pressure_and_airspeed message into a struct
 *
 * @param msg The message to decode
 * @param scaled_pressure_and_airspeed C-struct to decode the message contents into
 */
static inline void mavlink_msg_scaled_pressure_and_airspeed_decode(const mavlink_message_t* msg, mavlink_scaled_pressure_and_airspeed_t* scaled_pressure_and_airspeed)
{
#if MAVLINK_NEED_BYTE_SWAP
	scaled_pressure_and_airspeed->time_boot_ms = mavlink_msg_scaled_pressure_and_airspeed_get_time_boot_ms(msg);
	scaled_pressure_and_airspeed->press_abs = mavlink_msg_scaled_pressure_and_airspeed_get_press_abs(msg);
	scaled_pressure_and_airspeed->press_diff = mavlink_msg_scaled_pressure_and_airspeed_get_press_diff(msg);
	scaled_pressure_and_airspeed->airspeed = mavlink_msg_scaled_pressure_and_airspeed_get_airspeed(msg);
	scaled_pressure_and_airspeed->temperature = mavlink_msg_scaled_pressure_and_airspeed_get_temperature(msg);
#else
	memcpy(scaled_pressure_and_airspeed, _MAV_PAYLOAD(msg), 18);
#endif
}
