// MESSAGE LCM_TRANSPORT PACKING

#define MAVLINK_MSG_ID_LCM_TRANSPORT 223

typedef struct __mavlink_lcm_transport_t
{
 uint64_t utime; ///< Timestamp (milliseconds since system boot)
 uint32_t message_part_counter; ///< Some LCM messages are broken up into multiple mavlink messages.  This field denotes which part this is.
 uint32_t message_part_total; ///< Total number of payloads to combine to create the LCM message..
 uint32_t payload_size; ///< Size of this payload (likely full for most messages and then smaller for the last message.)
 uint16_t msg_id; ///< Unique id that will be the same for each mav message making up this LCM message
 char channel_name[21]; ///< Channel name
 char payload[62]; ///< Payload data that is all or part of an LCM message.
} mavlink_lcm_transport_t;

#define MAVLINK_MSG_ID_LCM_TRANSPORT_LEN 105
#define MAVLINK_MSG_ID_223_LEN 105

#define MAVLINK_MSG_LCM_TRANSPORT_FIELD_CHANNEL_NAME_LEN 21
#define MAVLINK_MSG_LCM_TRANSPORT_FIELD_PAYLOAD_LEN 62

#define MAVLINK_MESSAGE_INFO_LCM_TRANSPORT { \
	"LCM_TRANSPORT", \
	7, \
	{  { "utime", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_lcm_transport_t, utime) }, \
         { "message_part_counter", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_lcm_transport_t, message_part_counter) }, \
         { "message_part_total", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_lcm_transport_t, message_part_total) }, \
         { "payload_size", NULL, MAVLINK_TYPE_UINT32_T, 0, 16, offsetof(mavlink_lcm_transport_t, payload_size) }, \
         { "msg_id", NULL, MAVLINK_TYPE_UINT16_T, 0, 20, offsetof(mavlink_lcm_transport_t, msg_id) }, \
         { "channel_name", NULL, MAVLINK_TYPE_CHAR, 21, 22, offsetof(mavlink_lcm_transport_t, channel_name) }, \
         { "payload", NULL, MAVLINK_TYPE_CHAR, 62, 43, offsetof(mavlink_lcm_transport_t, payload) }, \
         } \
}


/**
 * @brief Pack a lcm_transport message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param utime Timestamp (milliseconds since system boot)
 * @param channel_name Channel name
 * @param msg_id Unique id that will be the same for each mav message making up this LCM message
 * @param message_part_counter Some LCM messages are broken up into multiple mavlink messages.  This field denotes which part this is.
 * @param message_part_total Total number of payloads to combine to create the LCM message..
 * @param payload_size Size of this payload (likely full for most messages and then smaller for the last message.)
 * @param payload Payload data that is all or part of an LCM message.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_lcm_transport_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t utime, const char *channel_name, uint16_t msg_id, uint32_t message_part_counter, uint32_t message_part_total, uint32_t payload_size, const char *payload)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[105];
	_mav_put_uint64_t(buf, 0, utime);
	_mav_put_uint32_t(buf, 8, message_part_counter);
	_mav_put_uint32_t(buf, 12, message_part_total);
	_mav_put_uint32_t(buf, 16, payload_size);
	_mav_put_uint16_t(buf, 20, msg_id);
	_mav_put_char_array(buf, 22, channel_name, 21);
	_mav_put_char_array(buf, 43, payload, 62);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 105);
#else
	mavlink_lcm_transport_t packet;
	packet.utime = utime;
	packet.message_part_counter = message_part_counter;
	packet.message_part_total = message_part_total;
	packet.payload_size = payload_size;
	packet.msg_id = msg_id;
	mav_array_memcpy(packet.channel_name, channel_name, sizeof(char)*21);
	mav_array_memcpy(packet.payload, payload, sizeof(char)*62);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 105);
#endif

	msg->msgid = MAVLINK_MSG_ID_LCM_TRANSPORT;
	return mavlink_finalize_message(msg, system_id, component_id, 105, 127);
}

/**
 * @brief Pack a lcm_transport message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param utime Timestamp (milliseconds since system boot)
 * @param channel_name Channel name
 * @param msg_id Unique id that will be the same for each mav message making up this LCM message
 * @param message_part_counter Some LCM messages are broken up into multiple mavlink messages.  This field denotes which part this is.
 * @param message_part_total Total number of payloads to combine to create the LCM message..
 * @param payload_size Size of this payload (likely full for most messages and then smaller for the last message.)
 * @param payload Payload data that is all or part of an LCM message.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_lcm_transport_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t utime,const char *channel_name,uint16_t msg_id,uint32_t message_part_counter,uint32_t message_part_total,uint32_t payload_size,const char *payload)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[105];
	_mav_put_uint64_t(buf, 0, utime);
	_mav_put_uint32_t(buf, 8, message_part_counter);
	_mav_put_uint32_t(buf, 12, message_part_total);
	_mav_put_uint32_t(buf, 16, payload_size);
	_mav_put_uint16_t(buf, 20, msg_id);
	_mav_put_char_array(buf, 22, channel_name, 21);
	_mav_put_char_array(buf, 43, payload, 62);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 105);
#else
	mavlink_lcm_transport_t packet;
	packet.utime = utime;
	packet.message_part_counter = message_part_counter;
	packet.message_part_total = message_part_total;
	packet.payload_size = payload_size;
	packet.msg_id = msg_id;
	mav_array_memcpy(packet.channel_name, channel_name, sizeof(char)*21);
	mav_array_memcpy(packet.payload, payload, sizeof(char)*62);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 105);
#endif

	msg->msgid = MAVLINK_MSG_ID_LCM_TRANSPORT;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 105, 127);
}

/**
 * @brief Encode a lcm_transport struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param lcm_transport C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_lcm_transport_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_lcm_transport_t* lcm_transport)
{
	return mavlink_msg_lcm_transport_pack(system_id, component_id, msg, lcm_transport->utime, lcm_transport->channel_name, lcm_transport->msg_id, lcm_transport->message_part_counter, lcm_transport->message_part_total, lcm_transport->payload_size, lcm_transport->payload);
}

/**
 * @brief Send a lcm_transport message
 * @param chan MAVLink channel to send the message
 *
 * @param utime Timestamp (milliseconds since system boot)
 * @param channel_name Channel name
 * @param msg_id Unique id that will be the same for each mav message making up this LCM message
 * @param message_part_counter Some LCM messages are broken up into multiple mavlink messages.  This field denotes which part this is.
 * @param message_part_total Total number of payloads to combine to create the LCM message..
 * @param payload_size Size of this payload (likely full for most messages and then smaller for the last message.)
 * @param payload Payload data that is all or part of an LCM message.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_lcm_transport_send(mavlink_channel_t chan, uint64_t utime, const char *channel_name, uint16_t msg_id, uint32_t message_part_counter, uint32_t message_part_total, uint32_t payload_size, const char *payload)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[105];
	_mav_put_uint64_t(buf, 0, utime);
	_mav_put_uint32_t(buf, 8, message_part_counter);
	_mav_put_uint32_t(buf, 12, message_part_total);
	_mav_put_uint32_t(buf, 16, payload_size);
	_mav_put_uint16_t(buf, 20, msg_id);
	_mav_put_char_array(buf, 22, channel_name, 21);
	_mav_put_char_array(buf, 43, payload, 62);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LCM_TRANSPORT, buf, 105, 127);
#else
	mavlink_lcm_transport_t packet;
	packet.utime = utime;
	packet.message_part_counter = message_part_counter;
	packet.message_part_total = message_part_total;
	packet.payload_size = payload_size;
	packet.msg_id = msg_id;
	mav_array_memcpy(packet.channel_name, channel_name, sizeof(char)*21);
	mav_array_memcpy(packet.payload, payload, sizeof(char)*62);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LCM_TRANSPORT, (const char *)&packet, 105, 127);
#endif
}

#endif

// MESSAGE LCM_TRANSPORT UNPACKING


/**
 * @brief Get field utime from lcm_transport message
 *
 * @return Timestamp (milliseconds since system boot)
 */
static inline uint64_t mavlink_msg_lcm_transport_get_utime(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field channel_name from lcm_transport message
 *
 * @return Channel name
 */
static inline uint16_t mavlink_msg_lcm_transport_get_channel_name(const mavlink_message_t* msg, char *channel_name)
{
	return _MAV_RETURN_char_array(msg, channel_name, 21,  22);
}

/**
 * @brief Get field msg_id from lcm_transport message
 *
 * @return Unique id that will be the same for each mav message making up this LCM message
 */
static inline uint16_t mavlink_msg_lcm_transport_get_msg_id(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  20);
}

/**
 * @brief Get field message_part_counter from lcm_transport message
 *
 * @return Some LCM messages are broken up into multiple mavlink messages.  This field denotes which part this is.
 */
static inline uint32_t mavlink_msg_lcm_transport_get_message_part_counter(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field message_part_total from lcm_transport message
 *
 * @return Total number of payloads to combine to create the LCM message..
 */
static inline uint32_t mavlink_msg_lcm_transport_get_message_part_total(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  12);
}

/**
 * @brief Get field payload_size from lcm_transport message
 *
 * @return Size of this payload (likely full for most messages and then smaller for the last message.)
 */
static inline uint32_t mavlink_msg_lcm_transport_get_payload_size(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  16);
}

/**
 * @brief Get field payload from lcm_transport message
 *
 * @return Payload data that is all or part of an LCM message.
 */
static inline uint16_t mavlink_msg_lcm_transport_get_payload(const mavlink_message_t* msg, char *payload)
{
	return _MAV_RETURN_char_array(msg, payload, 62,  43);
}

/**
 * @brief Decode a lcm_transport message into a struct
 *
 * @param msg The message to decode
 * @param lcm_transport C-struct to decode the message contents into
 */
static inline void mavlink_msg_lcm_transport_decode(const mavlink_message_t* msg, mavlink_lcm_transport_t* lcm_transport)
{
#if MAVLINK_NEED_BYTE_SWAP
	lcm_transport->utime = mavlink_msg_lcm_transport_get_utime(msg);
	lcm_transport->message_part_counter = mavlink_msg_lcm_transport_get_message_part_counter(msg);
	lcm_transport->message_part_total = mavlink_msg_lcm_transport_get_message_part_total(msg);
	lcm_transport->payload_size = mavlink_msg_lcm_transport_get_payload_size(msg);
	lcm_transport->msg_id = mavlink_msg_lcm_transport_get_msg_id(msg);
	mavlink_msg_lcm_transport_get_channel_name(msg, lcm_transport->channel_name);
	mavlink_msg_lcm_transport_get_payload(msg, lcm_transport->payload);
#else
	memcpy(lcm_transport, _MAV_PAYLOAD(msg), 105);
#endif
}
