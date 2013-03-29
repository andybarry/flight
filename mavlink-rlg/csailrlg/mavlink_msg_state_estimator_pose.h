// MESSAGE STATE_ESTIMATOR_POSE PACKING

#define MAVLINK_MSG_ID_STATE_ESTIMATOR_POSE 222

typedef struct __mavlink_state_estimator_pose_t
{
 uint32_t utime; ///< Timestamp (milliseconds since system boot)
 float x; ///< Position on the X axis.
 float y; ///< Position on the Y axis.
 float z; ///< Position on the Z axis.
 float velx; ///< Position on the X axis.
 float vely; ///< Position on the Y axis.
 float velz; ///< Position on the Z axis.
 float q1; ///< Orientation in quaternians (1st component).
 float q2; ///< Orientation in quaternians (2nd component).
 float q3; ///< Orientation in quaternians (3rd component).
 float q4; ///< Orientation in quaternians (4th component).
 float rotation_rate_x; ///< Rotation rate on the X axis
 float rotation_rate_y; ///< Rotation rate on the Y axis
 float rotation_rate_z; ///< Rotation rate on the Z axis
 float accelx; ///< Acceleration on the X axis
 float accely; ///< Acceleration on the Y axis
 float accelz; ///< Acceleration on the Z axis
} mavlink_state_estimator_pose_t;

#define MAVLINK_MSG_ID_STATE_ESTIMATOR_POSE_LEN 68
#define MAVLINK_MSG_ID_222_LEN 68



#define MAVLINK_MESSAGE_INFO_STATE_ESTIMATOR_POSE { \
	"STATE_ESTIMATOR_POSE", \
	17, \
	{  { "utime", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_state_estimator_pose_t, utime) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_state_estimator_pose_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_state_estimator_pose_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_state_estimator_pose_t, z) }, \
         { "velx", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_state_estimator_pose_t, velx) }, \
         { "vely", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_state_estimator_pose_t, vely) }, \
         { "velz", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_state_estimator_pose_t, velz) }, \
         { "q1", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_state_estimator_pose_t, q1) }, \
         { "q2", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_state_estimator_pose_t, q2) }, \
         { "q3", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_state_estimator_pose_t, q3) }, \
         { "q4", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_state_estimator_pose_t, q4) }, \
         { "rotation_rate_x", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_state_estimator_pose_t, rotation_rate_x) }, \
         { "rotation_rate_y", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_state_estimator_pose_t, rotation_rate_y) }, \
         { "rotation_rate_z", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_state_estimator_pose_t, rotation_rate_z) }, \
         { "accelx", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_state_estimator_pose_t, accelx) }, \
         { "accely", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_state_estimator_pose_t, accely) }, \
         { "accelz", NULL, MAVLINK_TYPE_FLOAT, 0, 64, offsetof(mavlink_state_estimator_pose_t, accelz) }, \
         } \
}


/**
 * @brief Pack a state_estimator_pose message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param utime Timestamp (milliseconds since system boot)
 * @param x Position on the X axis.
 * @param y Position on the Y axis.
 * @param z Position on the Z axis.
 * @param velx Position on the X axis.
 * @param vely Position on the Y axis.
 * @param velz Position on the Z axis.
 * @param q1 Orientation in quaternians (1st component).
 * @param q2 Orientation in quaternians (2nd component).
 * @param q3 Orientation in quaternians (3rd component).
 * @param q4 Orientation in quaternians (4th component).
 * @param rotation_rate_x Rotation rate on the X axis
 * @param rotation_rate_y Rotation rate on the Y axis
 * @param rotation_rate_z Rotation rate on the Z axis
 * @param accelx Acceleration on the X axis
 * @param accely Acceleration on the Y axis
 * @param accelz Acceleration on the Z axis
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_state_estimator_pose_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t utime, float x, float y, float z, float velx, float vely, float velz, float q1, float q2, float q3, float q4, float rotation_rate_x, float rotation_rate_y, float rotation_rate_z, float accelx, float accely, float accelz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[68];
	_mav_put_uint32_t(buf, 0, utime);
	_mav_put_float(buf, 4, x);
	_mav_put_float(buf, 8, y);
	_mav_put_float(buf, 12, z);
	_mav_put_float(buf, 16, velx);
	_mav_put_float(buf, 20, vely);
	_mav_put_float(buf, 24, velz);
	_mav_put_float(buf, 28, q1);
	_mav_put_float(buf, 32, q2);
	_mav_put_float(buf, 36, q3);
	_mav_put_float(buf, 40, q4);
	_mav_put_float(buf, 44, rotation_rate_x);
	_mav_put_float(buf, 48, rotation_rate_y);
	_mav_put_float(buf, 52, rotation_rate_z);
	_mav_put_float(buf, 56, accelx);
	_mav_put_float(buf, 60, accely);
	_mav_put_float(buf, 64, accelz);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 68);
#else
	mavlink_state_estimator_pose_t packet;
	packet.utime = utime;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.velx = velx;
	packet.vely = vely;
	packet.velz = velz;
	packet.q1 = q1;
	packet.q2 = q2;
	packet.q3 = q3;
	packet.q4 = q4;
	packet.rotation_rate_x = rotation_rate_x;
	packet.rotation_rate_y = rotation_rate_y;
	packet.rotation_rate_z = rotation_rate_z;
	packet.accelx = accelx;
	packet.accely = accely;
	packet.accelz = accelz;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 68);
#endif

	msg->msgid = MAVLINK_MSG_ID_STATE_ESTIMATOR_POSE;
	return mavlink_finalize_message(msg, system_id, component_id, 68, 69);
}

/**
 * @brief Pack a state_estimator_pose message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param utime Timestamp (milliseconds since system boot)
 * @param x Position on the X axis.
 * @param y Position on the Y axis.
 * @param z Position on the Z axis.
 * @param velx Position on the X axis.
 * @param vely Position on the Y axis.
 * @param velz Position on the Z axis.
 * @param q1 Orientation in quaternians (1st component).
 * @param q2 Orientation in quaternians (2nd component).
 * @param q3 Orientation in quaternians (3rd component).
 * @param q4 Orientation in quaternians (4th component).
 * @param rotation_rate_x Rotation rate on the X axis
 * @param rotation_rate_y Rotation rate on the Y axis
 * @param rotation_rate_z Rotation rate on the Z axis
 * @param accelx Acceleration on the X axis
 * @param accely Acceleration on the Y axis
 * @param accelz Acceleration on the Z axis
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_state_estimator_pose_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t utime,float x,float y,float z,float velx,float vely,float velz,float q1,float q2,float q3,float q4,float rotation_rate_x,float rotation_rate_y,float rotation_rate_z,float accelx,float accely,float accelz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[68];
	_mav_put_uint32_t(buf, 0, utime);
	_mav_put_float(buf, 4, x);
	_mav_put_float(buf, 8, y);
	_mav_put_float(buf, 12, z);
	_mav_put_float(buf, 16, velx);
	_mav_put_float(buf, 20, vely);
	_mav_put_float(buf, 24, velz);
	_mav_put_float(buf, 28, q1);
	_mav_put_float(buf, 32, q2);
	_mav_put_float(buf, 36, q3);
	_mav_put_float(buf, 40, q4);
	_mav_put_float(buf, 44, rotation_rate_x);
	_mav_put_float(buf, 48, rotation_rate_y);
	_mav_put_float(buf, 52, rotation_rate_z);
	_mav_put_float(buf, 56, accelx);
	_mav_put_float(buf, 60, accely);
	_mav_put_float(buf, 64, accelz);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 68);
#else
	mavlink_state_estimator_pose_t packet;
	packet.utime = utime;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.velx = velx;
	packet.vely = vely;
	packet.velz = velz;
	packet.q1 = q1;
	packet.q2 = q2;
	packet.q3 = q3;
	packet.q4 = q4;
	packet.rotation_rate_x = rotation_rate_x;
	packet.rotation_rate_y = rotation_rate_y;
	packet.rotation_rate_z = rotation_rate_z;
	packet.accelx = accelx;
	packet.accely = accely;
	packet.accelz = accelz;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 68);
#endif

	msg->msgid = MAVLINK_MSG_ID_STATE_ESTIMATOR_POSE;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 68, 69);
}

/**
 * @brief Encode a state_estimator_pose struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param state_estimator_pose C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_state_estimator_pose_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_state_estimator_pose_t* state_estimator_pose)
{
	return mavlink_msg_state_estimator_pose_pack(system_id, component_id, msg, state_estimator_pose->utime, state_estimator_pose->x, state_estimator_pose->y, state_estimator_pose->z, state_estimator_pose->velx, state_estimator_pose->vely, state_estimator_pose->velz, state_estimator_pose->q1, state_estimator_pose->q2, state_estimator_pose->q3, state_estimator_pose->q4, state_estimator_pose->rotation_rate_x, state_estimator_pose->rotation_rate_y, state_estimator_pose->rotation_rate_z, state_estimator_pose->accelx, state_estimator_pose->accely, state_estimator_pose->accelz);
}

/**
 * @brief Send a state_estimator_pose message
 * @param chan MAVLink channel to send the message
 *
 * @param utime Timestamp (milliseconds since system boot)
 * @param x Position on the X axis.
 * @param y Position on the Y axis.
 * @param z Position on the Z axis.
 * @param velx Position on the X axis.
 * @param vely Position on the Y axis.
 * @param velz Position on the Z axis.
 * @param q1 Orientation in quaternians (1st component).
 * @param q2 Orientation in quaternians (2nd component).
 * @param q3 Orientation in quaternians (3rd component).
 * @param q4 Orientation in quaternians (4th component).
 * @param rotation_rate_x Rotation rate on the X axis
 * @param rotation_rate_y Rotation rate on the Y axis
 * @param rotation_rate_z Rotation rate on the Z axis
 * @param accelx Acceleration on the X axis
 * @param accely Acceleration on the Y axis
 * @param accelz Acceleration on the Z axis
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_state_estimator_pose_send(mavlink_channel_t chan, uint32_t utime, float x, float y, float z, float velx, float vely, float velz, float q1, float q2, float q3, float q4, float rotation_rate_x, float rotation_rate_y, float rotation_rate_z, float accelx, float accely, float accelz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[68];
	_mav_put_uint32_t(buf, 0, utime);
	_mav_put_float(buf, 4, x);
	_mav_put_float(buf, 8, y);
	_mav_put_float(buf, 12, z);
	_mav_put_float(buf, 16, velx);
	_mav_put_float(buf, 20, vely);
	_mav_put_float(buf, 24, velz);
	_mav_put_float(buf, 28, q1);
	_mav_put_float(buf, 32, q2);
	_mav_put_float(buf, 36, q3);
	_mav_put_float(buf, 40, q4);
	_mav_put_float(buf, 44, rotation_rate_x);
	_mav_put_float(buf, 48, rotation_rate_y);
	_mav_put_float(buf, 52, rotation_rate_z);
	_mav_put_float(buf, 56, accelx);
	_mav_put_float(buf, 60, accely);
	_mav_put_float(buf, 64, accelz);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STATE_ESTIMATOR_POSE, buf, 68, 69);
#else
	mavlink_state_estimator_pose_t packet;
	packet.utime = utime;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.velx = velx;
	packet.vely = vely;
	packet.velz = velz;
	packet.q1 = q1;
	packet.q2 = q2;
	packet.q3 = q3;
	packet.q4 = q4;
	packet.rotation_rate_x = rotation_rate_x;
	packet.rotation_rate_y = rotation_rate_y;
	packet.rotation_rate_z = rotation_rate_z;
	packet.accelx = accelx;
	packet.accely = accely;
	packet.accelz = accelz;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STATE_ESTIMATOR_POSE, (const char *)&packet, 68, 69);
#endif
}

#endif

// MESSAGE STATE_ESTIMATOR_POSE UNPACKING


/**
 * @brief Get field utime from state_estimator_pose message
 *
 * @return Timestamp (milliseconds since system boot)
 */
static inline uint32_t mavlink_msg_state_estimator_pose_get_utime(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field x from state_estimator_pose message
 *
 * @return Position on the X axis.
 */
static inline float mavlink_msg_state_estimator_pose_get_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field y from state_estimator_pose message
 *
 * @return Position on the Y axis.
 */
static inline float mavlink_msg_state_estimator_pose_get_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field z from state_estimator_pose message
 *
 * @return Position on the Z axis.
 */
static inline float mavlink_msg_state_estimator_pose_get_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field velx from state_estimator_pose message
 *
 * @return Position on the X axis.
 */
static inline float mavlink_msg_state_estimator_pose_get_velx(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field vely from state_estimator_pose message
 *
 * @return Position on the Y axis.
 */
static inline float mavlink_msg_state_estimator_pose_get_vely(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field velz from state_estimator_pose message
 *
 * @return Position on the Z axis.
 */
static inline float mavlink_msg_state_estimator_pose_get_velz(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field q1 from state_estimator_pose message
 *
 * @return Orientation in quaternians (1st component).
 */
static inline float mavlink_msg_state_estimator_pose_get_q1(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field q2 from state_estimator_pose message
 *
 * @return Orientation in quaternians (2nd component).
 */
static inline float mavlink_msg_state_estimator_pose_get_q2(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field q3 from state_estimator_pose message
 *
 * @return Orientation in quaternians (3rd component).
 */
static inline float mavlink_msg_state_estimator_pose_get_q3(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field q4 from state_estimator_pose message
 *
 * @return Orientation in quaternians (4th component).
 */
static inline float mavlink_msg_state_estimator_pose_get_q4(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field rotation_rate_x from state_estimator_pose message
 *
 * @return Rotation rate on the X axis
 */
static inline float mavlink_msg_state_estimator_pose_get_rotation_rate_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field rotation_rate_y from state_estimator_pose message
 *
 * @return Rotation rate on the Y axis
 */
static inline float mavlink_msg_state_estimator_pose_get_rotation_rate_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field rotation_rate_z from state_estimator_pose message
 *
 * @return Rotation rate on the Z axis
 */
static inline float mavlink_msg_state_estimator_pose_get_rotation_rate_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  52);
}

/**
 * @brief Get field accelx from state_estimator_pose message
 *
 * @return Acceleration on the X axis
 */
static inline float mavlink_msg_state_estimator_pose_get_accelx(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  56);
}

/**
 * @brief Get field accely from state_estimator_pose message
 *
 * @return Acceleration on the Y axis
 */
static inline float mavlink_msg_state_estimator_pose_get_accely(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  60);
}

/**
 * @brief Get field accelz from state_estimator_pose message
 *
 * @return Acceleration on the Z axis
 */
static inline float mavlink_msg_state_estimator_pose_get_accelz(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  64);
}

/**
 * @brief Decode a state_estimator_pose message into a struct
 *
 * @param msg The message to decode
 * @param state_estimator_pose C-struct to decode the message contents into
 */
static inline void mavlink_msg_state_estimator_pose_decode(const mavlink_message_t* msg, mavlink_state_estimator_pose_t* state_estimator_pose)
{
#if MAVLINK_NEED_BYTE_SWAP
	state_estimator_pose->utime = mavlink_msg_state_estimator_pose_get_utime(msg);
	state_estimator_pose->x = mavlink_msg_state_estimator_pose_get_x(msg);
	state_estimator_pose->y = mavlink_msg_state_estimator_pose_get_y(msg);
	state_estimator_pose->z = mavlink_msg_state_estimator_pose_get_z(msg);
	state_estimator_pose->velx = mavlink_msg_state_estimator_pose_get_velx(msg);
	state_estimator_pose->vely = mavlink_msg_state_estimator_pose_get_vely(msg);
	state_estimator_pose->velz = mavlink_msg_state_estimator_pose_get_velz(msg);
	state_estimator_pose->q1 = mavlink_msg_state_estimator_pose_get_q1(msg);
	state_estimator_pose->q2 = mavlink_msg_state_estimator_pose_get_q2(msg);
	state_estimator_pose->q3 = mavlink_msg_state_estimator_pose_get_q3(msg);
	state_estimator_pose->q4 = mavlink_msg_state_estimator_pose_get_q4(msg);
	state_estimator_pose->rotation_rate_x = mavlink_msg_state_estimator_pose_get_rotation_rate_x(msg);
	state_estimator_pose->rotation_rate_y = mavlink_msg_state_estimator_pose_get_rotation_rate_y(msg);
	state_estimator_pose->rotation_rate_z = mavlink_msg_state_estimator_pose_get_rotation_rate_z(msg);
	state_estimator_pose->accelx = mavlink_msg_state_estimator_pose_get_accelx(msg);
	state_estimator_pose->accely = mavlink_msg_state_estimator_pose_get_accely(msg);
	state_estimator_pose->accelz = mavlink_msg_state_estimator_pose_get_accelz(msg);
#else
	memcpy(state_estimator_pose, _MAV_PAYLOAD(msg), 68);
#endif
}
