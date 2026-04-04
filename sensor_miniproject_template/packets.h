/*
 * packets.h
 * Studio 13: Sensor Mini-Project
 *
 * TPacket protocol: enums, struct, and framing constants.
 * This file must be kept in sync with the constants in pi_sensor.py.
 */

#pragma once

#include <stdint.h>

#define SERVO_BASE 0
#define SERVO_SHOULDER 1
#define SERVO_ELBOW 2
#define SERVO_GRIPPER 3
#define SERVO_SPEED 4

// =============================================================
// TPacket protocol
// =============================================================

typedef enum {
    PACKET_TYPE_COMMAND  = 0,
    PACKET_TYPE_RESPONSE = 1,
    PACKET_TYPE_MESSAGE  = 2,
} TPacketType;

typedef enum {
    COMMAND_ESTOP = 0,
    COMMAND_COLOUR = 1,
    COMMAND_GO = 2,
    COMMAND_CW = 3,
    COMMAND_CCW = 4,
    COMMAND_BACK = 5,
    COMMAND_STOP = 6,
    COMMAND_FASTER = 7,
    COMMAND_SLOWER = 8,
    COMMAND_ARM_MOVE = 10
} TCommandType;

typedef enum {
    RESP_OK     = 0,
    RESP_STATUS = 1,
    RESP_COLOUR = 2,
} TResponseType;

typedef enum {
    STATE_RUNNING = 0,
    STATE_STOPPED = 1,
} TState;

typedef struct {
    uint8_t  packetType;
    uint8_t  command;
    uint8_t  dummy[2];
    char     data[32];
    uint32_t params[16];
} TPacket;

// =============================================================
// Framing constants
// =============================================================

#define MAGIC_HI        0xDE
#define MAGIC_LO        0xAD
#define TPACKET_SIZE    ((uint8_t)sizeof(TPacket))   // 100 bytes
#define FRAME_SIZE      (2 + TPACKET_SIZE + 1)       // 103 bytes
