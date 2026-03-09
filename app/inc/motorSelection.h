#pragma once

/**
  *****************************************************************************
  * @file    motorSelection.h
  * @brief   Motor configuration table for the single-MCU daisy-chain node.
  *
  *          MOTOR_MAX is the compile-time ceiling (max possible motors = 6).
  *          The actual number of motors present is discovered at runtime via
  *          a servo-bus ping scan and stored in g_active_motor_count.
  *
  *  To add or reconfigure motors:
  *    • Edit g_motor_configs[] in motor_config.c for permanent changes.
  *    • Send an RPi_Reconfig CAN frame (0x300) for live reconfiguration.
  *
  *  Servo ID → Joint mapping
  *  ------------------------
  *  Servo bus ID 1 ↔ Joint 1 ↔ RPi_Command_1 / MCU_Status_1
  *  Servo bus ID 2 ↔ Joint 2 ↔ RPi_Command_2 / MCU_Status_2   ... etc.
  *  (IDs must be pre-programmed into the physical servos.)
  *****************************************************************************
  */

#include <stdbool.h>
#include "hiwonder_bus_servo.h"
#include "omni_robot.h"  // CAN frame ID / length constants

/* ── Compile-time motor ceiling ──────────────────────────────────── */
#define MOTOR_MAX   6   /**< Maximum number of servos on the bus */

/* ── CAN message for live reconfiguration ────────────────────────── */
#define MOTOR_RECONFIG_CAN_ID   OMNI_ROBOT_R_PI_RECONFIG_FRAME_ID  /* 0x300 */

/* ── Canonical CAN payload types ──────────────────────────────────
 *
 *  All six RPi_Command structs are layout-identical (same fields, same
 *  sizes).  Likewise for the six MCU_Status structs.  We use the _1
 *  variants as the canonical C type and cast where necessary.
 * ─────────────────────────────────────────────────────────────────── */
typedef struct omni_robot_r_pi_command_1_t  motor_cmd_t;
typedef struct omni_robot_mcu_status_1_t    motor_status_t;

/* ── Function-pointer types for CAN pack / unpack ─────────────────── */
typedef int (*motor_cmd_unpack_fn)(motor_cmd_t       *dst,
                                   const uint8_t     *data,
                                   size_t             size);

typedef int (*motor_status_pack_fn)(uint8_t              *dst,
                                    const motor_status_t *src,
                                    size_t                size);

/* ── Per-motor compile-time descriptor ────────────────────────────── */
typedef struct {
    uint8_t                servo_id;        // unique servo bus ID (1–6)
    hiwonder_servo_spec_t  spec;            // physical angular range
    const char            *name;            // human-readable label for logs

    uint32_t               can_cmd_id;      // CAN std-ID for incoming RPi_Command
    uint32_t               can_status_id;   // CAN std-ID for outgoing MCU_Status
    uint8_t                can_status_len;  // DLC of MCU_Status frame (bytes)

    motor_cmd_unpack_fn    cmd_unpack;      // cantools-generated unpack fn
    motor_status_pack_fn   status_pack;     // cantools-generated pack fn
} motor_config_t;

/* ── Global config table (defined in motor_config.c) ─────────────── */
extern const motor_config_t g_motor_configs[MOTOR_MAX];

/* ── Runtime motor presence state (defined in servoMotor.c) ──────── */
/**
 * g_servo_present[i] is true iff the servo at g_motor_configs[i].servo_id
 * responded during the last scan.  Index i is 0-based (servo ID − 1).
 *
 * g_active_motor_count is the total number of responding servos.
 *
 * Both are written exclusively by the servoMotorTask and may be read
 * by other tasks for status purposes (no mutex needed for single-byte reads).
 */
extern volatile uint8_t g_active_motor_count;
extern volatile bool    g_servo_present[MOTOR_MAX];
