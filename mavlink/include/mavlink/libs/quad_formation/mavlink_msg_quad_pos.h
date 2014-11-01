// MESSAGE QUAD_POS PACKING

#define MAVLINK_MSG_ID_QUAD_POS 160

typedef struct __mavlink_quad_pos_t
{
 int16_t x; ///< x-axis (int16_t)
 int16_t y; ///< y-axis (int16_t)
 int16_t z; ///< z-axis (int16_t)
 uint8_t pos_no; ///< Position number
} mavlink_quad_pos_t;

#define MAVLINK_MSG_ID_QUAD_POS_LEN 7
#define MAVLINK_MSG_ID_160_LEN 7

#define MAVLINK_MSG_ID_QUAD_POS_CRC 32
#define MAVLINK_MSG_ID_160_CRC 32



#define MAVLINK_MESSAGE_INFO_QUAD_POS { \
	"QUAD_POS", \
	4, \
	{  { "x", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_quad_pos_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_INT16_T, 0, 2, offsetof(mavlink_quad_pos_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_quad_pos_t, z) }, \
         { "pos_no", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_quad_pos_t, pos_no) }, \
         } \
}


/**
 * @brief Pack a quad_pos message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param pos_no Position number
 * @param x x-axis (int16_t)
 * @param y y-axis (int16_t)
 * @param z z-axis (int16_t)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_quad_pos_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t pos_no, int16_t x, int16_t y, int16_t z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_QUAD_POS_LEN];
	_mav_put_int16_t(buf, 0, x);
	_mav_put_int16_t(buf, 2, y);
	_mav_put_int16_t(buf, 4, z);
	_mav_put_uint8_t(buf, 6, pos_no);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_QUAD_POS_LEN);
#else
	mavlink_quad_pos_t packet;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.pos_no = pos_no;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_QUAD_POS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_QUAD_POS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_QUAD_POS_LEN, MAVLINK_MSG_ID_QUAD_POS_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_QUAD_POS_LEN);
#endif
}

/**
 * @brief Pack a quad_pos message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param pos_no Position number
 * @param x x-axis (int16_t)
 * @param y y-axis (int16_t)
 * @param z z-axis (int16_t)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_quad_pos_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t pos_no,int16_t x,int16_t y,int16_t z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_QUAD_POS_LEN];
	_mav_put_int16_t(buf, 0, x);
	_mav_put_int16_t(buf, 2, y);
	_mav_put_int16_t(buf, 4, z);
	_mav_put_uint8_t(buf, 6, pos_no);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_QUAD_POS_LEN);
#else
	mavlink_quad_pos_t packet;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.pos_no = pos_no;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_QUAD_POS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_QUAD_POS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_QUAD_POS_LEN, MAVLINK_MSG_ID_QUAD_POS_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_QUAD_POS_LEN);
#endif
}

/**
 * @brief Encode a quad_pos struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param quad_pos C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_quad_pos_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_quad_pos_t* quad_pos)
{
	return mavlink_msg_quad_pos_pack(system_id, component_id, msg, quad_pos->pos_no, quad_pos->x, quad_pos->y, quad_pos->z);
}

/**
 * @brief Encode a quad_pos struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param quad_pos C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_quad_pos_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_quad_pos_t* quad_pos)
{
	return mavlink_msg_quad_pos_pack_chan(system_id, component_id, chan, msg, quad_pos->pos_no, quad_pos->x, quad_pos->y, quad_pos->z);
}

/**
 * @brief Send a quad_pos message
 * @param chan MAVLink channel to send the message
 *
 * @param pos_no Position number
 * @param x x-axis (int16_t)
 * @param y y-axis (int16_t)
 * @param z z-axis (int16_t)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_quad_pos_send(mavlink_channel_t chan, uint8_t pos_no, int16_t x, int16_t y, int16_t z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_QUAD_POS_LEN];
	_mav_put_int16_t(buf, 0, x);
	_mav_put_int16_t(buf, 2, y);
	_mav_put_int16_t(buf, 4, z);
	_mav_put_uint8_t(buf, 6, pos_no);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_QUAD_POS, buf, MAVLINK_MSG_ID_QUAD_POS_LEN, MAVLINK_MSG_ID_QUAD_POS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_QUAD_POS, buf, MAVLINK_MSG_ID_QUAD_POS_LEN);
#endif
#else
	mavlink_quad_pos_t packet;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.pos_no = pos_no;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_QUAD_POS, (const char *)&packet, MAVLINK_MSG_ID_QUAD_POS_LEN, MAVLINK_MSG_ID_QUAD_POS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_QUAD_POS, (const char *)&packet, MAVLINK_MSG_ID_QUAD_POS_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_QUAD_POS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_quad_pos_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t pos_no, int16_t x, int16_t y, int16_t z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_int16_t(buf, 0, x);
	_mav_put_int16_t(buf, 2, y);
	_mav_put_int16_t(buf, 4, z);
	_mav_put_uint8_t(buf, 6, pos_no);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_QUAD_POS, buf, MAVLINK_MSG_ID_QUAD_POS_LEN, MAVLINK_MSG_ID_QUAD_POS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_QUAD_POS, buf, MAVLINK_MSG_ID_QUAD_POS_LEN);
#endif
#else
	mavlink_quad_pos_t *packet = (mavlink_quad_pos_t *)msgbuf;
	packet->x = x;
	packet->y = y;
	packet->z = z;
	packet->pos_no = pos_no;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_QUAD_POS, (const char *)packet, MAVLINK_MSG_ID_QUAD_POS_LEN, MAVLINK_MSG_ID_QUAD_POS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_QUAD_POS, (const char *)packet, MAVLINK_MSG_ID_QUAD_POS_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE QUAD_POS UNPACKING


/**
 * @brief Get field pos_no from quad_pos message
 *
 * @return Position number
 */
static inline uint8_t mavlink_msg_quad_pos_get_pos_no(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field x from quad_pos message
 *
 * @return x-axis (int16_t)
 */
static inline int16_t mavlink_msg_quad_pos_get_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  0);
}

/**
 * @brief Get field y from quad_pos message
 *
 * @return y-axis (int16_t)
 */
static inline int16_t mavlink_msg_quad_pos_get_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  2);
}

/**
 * @brief Get field z from quad_pos message
 *
 * @return z-axis (int16_t)
 */
static inline int16_t mavlink_msg_quad_pos_get_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  4);
}

/**
 * @brief Decode a quad_pos message into a struct
 *
 * @param msg The message to decode
 * @param quad_pos C-struct to decode the message contents into
 */
static inline void mavlink_msg_quad_pos_decode(const mavlink_message_t* msg, mavlink_quad_pos_t* quad_pos)
{
#if MAVLINK_NEED_BYTE_SWAP
	quad_pos->x = mavlink_msg_quad_pos_get_x(msg);
	quad_pos->y = mavlink_msg_quad_pos_get_y(msg);
	quad_pos->z = mavlink_msg_quad_pos_get_z(msg);
	quad_pos->pos_no = mavlink_msg_quad_pos_get_pos_no(msg);
#else
	memcpy(quad_pos, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_QUAD_POS_LEN);
#endif
}
