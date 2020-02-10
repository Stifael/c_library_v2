#pragma once
// MESSAGE SET_ATTITUDE_VELOCITY_Z_TARGET PACKING

#define MAVLINK_MSG_ID_SET_ATTITUDE_VELOCITY_Z_TARGET 9030

MAVPACKED(
typedef struct __mavlink_set_attitude_velocity_z_target_t {
 uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot).*/
 float roll; /*< [rad] Euler roll in rad*/
 float pitch; /*< [rad] Euler pitch in rad*/
 float body_yaw_rate; /*< [rad/s] Body yaw rate*/
 float velocity_z; /*< [m/s] Velocity in z-direction NED frame.*/
 uint8_t target_system; /*<  System ID*/
 uint8_t target_component; /*<  Component ID*/
 uint8_t hold_alt; /*<  Boolean indicating if 0-velocity should result in altitude hold.*/
}) mavlink_set_attitude_velocity_z_target_t;

#define MAVLINK_MSG_ID_SET_ATTITUDE_VELOCITY_Z_TARGET_LEN 23
#define MAVLINK_MSG_ID_SET_ATTITUDE_VELOCITY_Z_TARGET_MIN_LEN 23
#define MAVLINK_MSG_ID_9030_LEN 23
#define MAVLINK_MSG_ID_9030_MIN_LEN 23

#define MAVLINK_MSG_ID_SET_ATTITUDE_VELOCITY_Z_TARGET_CRC 215
#define MAVLINK_MSG_ID_9030_CRC 215



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SET_ATTITUDE_VELOCITY_Z_TARGET { \
    9030, \
    "SET_ATTITUDE_VELOCITY_Z_TARGET", \
    8, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_set_attitude_velocity_z_target_t, time_boot_ms) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_set_attitude_velocity_z_target_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 21, offsetof(mavlink_set_attitude_velocity_z_target_t, target_component) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_set_attitude_velocity_z_target_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_set_attitude_velocity_z_target_t, pitch) }, \
         { "body_yaw_rate", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_set_attitude_velocity_z_target_t, body_yaw_rate) }, \
         { "velocity_z", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_set_attitude_velocity_z_target_t, velocity_z) }, \
         { "hold_alt", NULL, MAVLINK_TYPE_UINT8_T, 0, 22, offsetof(mavlink_set_attitude_velocity_z_target_t, hold_alt) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SET_ATTITUDE_VELOCITY_Z_TARGET { \
    "SET_ATTITUDE_VELOCITY_Z_TARGET", \
    8, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_set_attitude_velocity_z_target_t, time_boot_ms) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_set_attitude_velocity_z_target_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 21, offsetof(mavlink_set_attitude_velocity_z_target_t, target_component) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_set_attitude_velocity_z_target_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_set_attitude_velocity_z_target_t, pitch) }, \
         { "body_yaw_rate", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_set_attitude_velocity_z_target_t, body_yaw_rate) }, \
         { "velocity_z", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_set_attitude_velocity_z_target_t, velocity_z) }, \
         { "hold_alt", NULL, MAVLINK_TYPE_UINT8_T, 0, 22, offsetof(mavlink_set_attitude_velocity_z_target_t, hold_alt) }, \
         } \
}
#endif

/**
 * @brief Pack a set_attitude_velocity_z_target message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param roll [rad] Euler roll in rad
 * @param pitch [rad] Euler pitch in rad
 * @param body_yaw_rate [rad/s] Body yaw rate
 * @param velocity_z [m/s] Velocity in z-direction NED frame.
 * @param hold_alt  Boolean indicating if 0-velocity should result in altitude hold.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_attitude_velocity_z_target_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_boot_ms, uint8_t target_system, uint8_t target_component, float roll, float pitch, float body_yaw_rate, float velocity_z, uint8_t hold_alt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_ATTITUDE_VELOCITY_Z_TARGET_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, roll);
    _mav_put_float(buf, 8, pitch);
    _mav_put_float(buf, 12, body_yaw_rate);
    _mav_put_float(buf, 16, velocity_z);
    _mav_put_uint8_t(buf, 20, target_system);
    _mav_put_uint8_t(buf, 21, target_component);
    _mav_put_uint8_t(buf, 22, hold_alt);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_ATTITUDE_VELOCITY_Z_TARGET_LEN);
#else
    mavlink_set_attitude_velocity_z_target_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.body_yaw_rate = body_yaw_rate;
    packet.velocity_z = velocity_z;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.hold_alt = hold_alt;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_ATTITUDE_VELOCITY_Z_TARGET_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SET_ATTITUDE_VELOCITY_Z_TARGET;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SET_ATTITUDE_VELOCITY_Z_TARGET_MIN_LEN, MAVLINK_MSG_ID_SET_ATTITUDE_VELOCITY_Z_TARGET_LEN, MAVLINK_MSG_ID_SET_ATTITUDE_VELOCITY_Z_TARGET_CRC);
}

/**
 * @brief Pack a set_attitude_velocity_z_target message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param roll [rad] Euler roll in rad
 * @param pitch [rad] Euler pitch in rad
 * @param body_yaw_rate [rad/s] Body yaw rate
 * @param velocity_z [m/s] Velocity in z-direction NED frame.
 * @param hold_alt  Boolean indicating if 0-velocity should result in altitude hold.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_attitude_velocity_z_target_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_boot_ms,uint8_t target_system,uint8_t target_component,float roll,float pitch,float body_yaw_rate,float velocity_z,uint8_t hold_alt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_ATTITUDE_VELOCITY_Z_TARGET_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, roll);
    _mav_put_float(buf, 8, pitch);
    _mav_put_float(buf, 12, body_yaw_rate);
    _mav_put_float(buf, 16, velocity_z);
    _mav_put_uint8_t(buf, 20, target_system);
    _mav_put_uint8_t(buf, 21, target_component);
    _mav_put_uint8_t(buf, 22, hold_alt);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_ATTITUDE_VELOCITY_Z_TARGET_LEN);
#else
    mavlink_set_attitude_velocity_z_target_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.body_yaw_rate = body_yaw_rate;
    packet.velocity_z = velocity_z;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.hold_alt = hold_alt;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_ATTITUDE_VELOCITY_Z_TARGET_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SET_ATTITUDE_VELOCITY_Z_TARGET;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SET_ATTITUDE_VELOCITY_Z_TARGET_MIN_LEN, MAVLINK_MSG_ID_SET_ATTITUDE_VELOCITY_Z_TARGET_LEN, MAVLINK_MSG_ID_SET_ATTITUDE_VELOCITY_Z_TARGET_CRC);
}

/**
 * @brief Encode a set_attitude_velocity_z_target struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param set_attitude_velocity_z_target C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_attitude_velocity_z_target_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_set_attitude_velocity_z_target_t* set_attitude_velocity_z_target)
{
    return mavlink_msg_set_attitude_velocity_z_target_pack(system_id, component_id, msg, set_attitude_velocity_z_target->time_boot_ms, set_attitude_velocity_z_target->target_system, set_attitude_velocity_z_target->target_component, set_attitude_velocity_z_target->roll, set_attitude_velocity_z_target->pitch, set_attitude_velocity_z_target->body_yaw_rate, set_attitude_velocity_z_target->velocity_z, set_attitude_velocity_z_target->hold_alt);
}

/**
 * @brief Encode a set_attitude_velocity_z_target struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param set_attitude_velocity_z_target C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_attitude_velocity_z_target_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_set_attitude_velocity_z_target_t* set_attitude_velocity_z_target)
{
    return mavlink_msg_set_attitude_velocity_z_target_pack_chan(system_id, component_id, chan, msg, set_attitude_velocity_z_target->time_boot_ms, set_attitude_velocity_z_target->target_system, set_attitude_velocity_z_target->target_component, set_attitude_velocity_z_target->roll, set_attitude_velocity_z_target->pitch, set_attitude_velocity_z_target->body_yaw_rate, set_attitude_velocity_z_target->velocity_z, set_attitude_velocity_z_target->hold_alt);
}

/**
 * @brief Send a set_attitude_velocity_z_target message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param roll [rad] Euler roll in rad
 * @param pitch [rad] Euler pitch in rad
 * @param body_yaw_rate [rad/s] Body yaw rate
 * @param velocity_z [m/s] Velocity in z-direction NED frame.
 * @param hold_alt  Boolean indicating if 0-velocity should result in altitude hold.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_set_attitude_velocity_z_target_send(mavlink_channel_t chan, uint32_t time_boot_ms, uint8_t target_system, uint8_t target_component, float roll, float pitch, float body_yaw_rate, float velocity_z, uint8_t hold_alt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_ATTITUDE_VELOCITY_Z_TARGET_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, roll);
    _mav_put_float(buf, 8, pitch);
    _mav_put_float(buf, 12, body_yaw_rate);
    _mav_put_float(buf, 16, velocity_z);
    _mav_put_uint8_t(buf, 20, target_system);
    _mav_put_uint8_t(buf, 21, target_component);
    _mav_put_uint8_t(buf, 22, hold_alt);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_ATTITUDE_VELOCITY_Z_TARGET, buf, MAVLINK_MSG_ID_SET_ATTITUDE_VELOCITY_Z_TARGET_MIN_LEN, MAVLINK_MSG_ID_SET_ATTITUDE_VELOCITY_Z_TARGET_LEN, MAVLINK_MSG_ID_SET_ATTITUDE_VELOCITY_Z_TARGET_CRC);
#else
    mavlink_set_attitude_velocity_z_target_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.body_yaw_rate = body_yaw_rate;
    packet.velocity_z = velocity_z;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.hold_alt = hold_alt;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_ATTITUDE_VELOCITY_Z_TARGET, (const char *)&packet, MAVLINK_MSG_ID_SET_ATTITUDE_VELOCITY_Z_TARGET_MIN_LEN, MAVLINK_MSG_ID_SET_ATTITUDE_VELOCITY_Z_TARGET_LEN, MAVLINK_MSG_ID_SET_ATTITUDE_VELOCITY_Z_TARGET_CRC);
#endif
}

/**
 * @brief Send a set_attitude_velocity_z_target message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_set_attitude_velocity_z_target_send_struct(mavlink_channel_t chan, const mavlink_set_attitude_velocity_z_target_t* set_attitude_velocity_z_target)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_set_attitude_velocity_z_target_send(chan, set_attitude_velocity_z_target->time_boot_ms, set_attitude_velocity_z_target->target_system, set_attitude_velocity_z_target->target_component, set_attitude_velocity_z_target->roll, set_attitude_velocity_z_target->pitch, set_attitude_velocity_z_target->body_yaw_rate, set_attitude_velocity_z_target->velocity_z, set_attitude_velocity_z_target->hold_alt);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_ATTITUDE_VELOCITY_Z_TARGET, (const char *)set_attitude_velocity_z_target, MAVLINK_MSG_ID_SET_ATTITUDE_VELOCITY_Z_TARGET_MIN_LEN, MAVLINK_MSG_ID_SET_ATTITUDE_VELOCITY_Z_TARGET_LEN, MAVLINK_MSG_ID_SET_ATTITUDE_VELOCITY_Z_TARGET_CRC);
#endif
}

#if MAVLINK_MSG_ID_SET_ATTITUDE_VELOCITY_Z_TARGET_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_set_attitude_velocity_z_target_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, uint8_t target_system, uint8_t target_component, float roll, float pitch, float body_yaw_rate, float velocity_z, uint8_t hold_alt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, roll);
    _mav_put_float(buf, 8, pitch);
    _mav_put_float(buf, 12, body_yaw_rate);
    _mav_put_float(buf, 16, velocity_z);
    _mav_put_uint8_t(buf, 20, target_system);
    _mav_put_uint8_t(buf, 21, target_component);
    _mav_put_uint8_t(buf, 22, hold_alt);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_ATTITUDE_VELOCITY_Z_TARGET, buf, MAVLINK_MSG_ID_SET_ATTITUDE_VELOCITY_Z_TARGET_MIN_LEN, MAVLINK_MSG_ID_SET_ATTITUDE_VELOCITY_Z_TARGET_LEN, MAVLINK_MSG_ID_SET_ATTITUDE_VELOCITY_Z_TARGET_CRC);
#else
    mavlink_set_attitude_velocity_z_target_t *packet = (mavlink_set_attitude_velocity_z_target_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->roll = roll;
    packet->pitch = pitch;
    packet->body_yaw_rate = body_yaw_rate;
    packet->velocity_z = velocity_z;
    packet->target_system = target_system;
    packet->target_component = target_component;
    packet->hold_alt = hold_alt;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_ATTITUDE_VELOCITY_Z_TARGET, (const char *)packet, MAVLINK_MSG_ID_SET_ATTITUDE_VELOCITY_Z_TARGET_MIN_LEN, MAVLINK_MSG_ID_SET_ATTITUDE_VELOCITY_Z_TARGET_LEN, MAVLINK_MSG_ID_SET_ATTITUDE_VELOCITY_Z_TARGET_CRC);
#endif
}
#endif

#endif

// MESSAGE SET_ATTITUDE_VELOCITY_Z_TARGET UNPACKING


/**
 * @brief Get field time_boot_ms from set_attitude_velocity_z_target message
 *
 * @return [ms] Timestamp (time since system boot).
 */
static inline uint32_t mavlink_msg_set_attitude_velocity_z_target_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field target_system from set_attitude_velocity_z_target message
 *
 * @return  System ID
 */
static inline uint8_t mavlink_msg_set_attitude_velocity_z_target_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  20);
}

/**
 * @brief Get field target_component from set_attitude_velocity_z_target message
 *
 * @return  Component ID
 */
static inline uint8_t mavlink_msg_set_attitude_velocity_z_target_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  21);
}

/**
 * @brief Get field roll from set_attitude_velocity_z_target message
 *
 * @return [rad] Euler roll in rad
 */
static inline float mavlink_msg_set_attitude_velocity_z_target_get_roll(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field pitch from set_attitude_velocity_z_target message
 *
 * @return [rad] Euler pitch in rad
 */
static inline float mavlink_msg_set_attitude_velocity_z_target_get_pitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field body_yaw_rate from set_attitude_velocity_z_target message
 *
 * @return [rad/s] Body yaw rate
 */
static inline float mavlink_msg_set_attitude_velocity_z_target_get_body_yaw_rate(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field velocity_z from set_attitude_velocity_z_target message
 *
 * @return [m/s] Velocity in z-direction NED frame.
 */
static inline float mavlink_msg_set_attitude_velocity_z_target_get_velocity_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field hold_alt from set_attitude_velocity_z_target message
 *
 * @return  Boolean indicating if 0-velocity should result in altitude hold.
 */
static inline uint8_t mavlink_msg_set_attitude_velocity_z_target_get_hold_alt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  22);
}

/**
 * @brief Decode a set_attitude_velocity_z_target message into a struct
 *
 * @param msg The message to decode
 * @param set_attitude_velocity_z_target C-struct to decode the message contents into
 */
static inline void mavlink_msg_set_attitude_velocity_z_target_decode(const mavlink_message_t* msg, mavlink_set_attitude_velocity_z_target_t* set_attitude_velocity_z_target)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    set_attitude_velocity_z_target->time_boot_ms = mavlink_msg_set_attitude_velocity_z_target_get_time_boot_ms(msg);
    set_attitude_velocity_z_target->roll = mavlink_msg_set_attitude_velocity_z_target_get_roll(msg);
    set_attitude_velocity_z_target->pitch = mavlink_msg_set_attitude_velocity_z_target_get_pitch(msg);
    set_attitude_velocity_z_target->body_yaw_rate = mavlink_msg_set_attitude_velocity_z_target_get_body_yaw_rate(msg);
    set_attitude_velocity_z_target->velocity_z = mavlink_msg_set_attitude_velocity_z_target_get_velocity_z(msg);
    set_attitude_velocity_z_target->target_system = mavlink_msg_set_attitude_velocity_z_target_get_target_system(msg);
    set_attitude_velocity_z_target->target_component = mavlink_msg_set_attitude_velocity_z_target_get_target_component(msg);
    set_attitude_velocity_z_target->hold_alt = mavlink_msg_set_attitude_velocity_z_target_get_hold_alt(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SET_ATTITUDE_VELOCITY_Z_TARGET_LEN? msg->len : MAVLINK_MSG_ID_SET_ATTITUDE_VELOCITY_Z_TARGET_LEN;
        memset(set_attitude_velocity_z_target, 0, MAVLINK_MSG_ID_SET_ATTITUDE_VELOCITY_Z_TARGET_LEN);
    memcpy(set_attitude_velocity_z_target, _MAV_PAYLOAD(msg), len);
#endif
}
