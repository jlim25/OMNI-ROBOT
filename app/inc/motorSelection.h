#pragma once

/**
  *****************************************************************************
  * @file    motorSelection.h
  * @brief   Compile-time motor selection for a single-motor MCU node.
  *
  *          Each MCU in the system drives exactly one servo.  Set
  *          MOTOR_SELECTION to the joint this MCU is responsible for.
  *          Everything else (hardware ID, angular spec) is derived here.
  *
  *  Usage
  *  -----
  *  Uncomment exactly ONE of the MOTOR_SELECTION lines below, or supply
  *  the definition as a compiler flag: -DMOTOR_SELECTION=MOTOR_JOINT_1
  *****************************************************************************
  */

#include "hiwonder_bus_servo.h"
#include "omni_robot.h"  // per-motor CAN frame ID constants

/* ── Available motor identifiers ────────────────────────────────── */
#define MOTOR_JOINT_1   1
#define MOTOR_JOINT_2   2
#define MOTOR_JOINT_3   3
#define MOTOR_JOINT_4   4

/* ── Select the motor for this MCU (uncomment exactly one) ───────── */
#define MOTOR_SELECTION   MOTOR_JOINT_1
// #define MOTOR_SELECTION   MOTOR_JOINT_2
// #define MOTOR_SELECTION   MOTOR_JOINT_3
// #define MOTOR_SELECTION   MOTOR_JOINT_4

/* ── Per-motor configuration ─────────────────────────────────────
 *
 *  MOTOR_ID   – servo hardware ID on the half-duplex bus (1–253)
 *  MOTOR_SPEC – hiwonder_servo_spec_t initializer for angular range
 *  MOTOR_NAME – human-readable label used in log messages
 *
 * ──────────────────────────────────────────────────────────────── */
#if   MOTOR_SELECTION == MOTOR_JOINT_1
    #define MOTOR_ID              1
    #define MOTOR_SPEC            ((hiwonder_servo_spec_t)HTD_85H_SPEC)
    #define MOTOR_NAME            "Base-1 (240 deg)"
    #define MOTOR_CAN_CMD_ID      OMNI_ROBOT_R_PI_COMMAND_1_FRAME_ID
    #define MOTOR_CAN_STATUS_ID   OMNI_ROBOT_MCU_STATUS_1_FRAME_ID
    #define MOTOR_CAN_CMD_T       omni_robot_r_pi_command_1_t
    #define MOTOR_CAN_STATUS_T    omni_robot_mcu_status_1_t
    #define MOTOR_CAN_CMD_UNPACK  omni_robot_r_pi_command_1_unpack
    #define MOTOR_CAN_STATUS_PACK omni_robot_mcu_status_1_pack
    #define MOTOR_CAN_STATUS_LENGTH OMNI_ROBOT_MCU_STATUS_1_LENGTH

#elif MOTOR_SELECTION == MOTOR_JOINT_2
    #define MOTOR_ID              2
    #define MOTOR_SPEC            ((hiwonder_servo_spec_t)HTD_85H_SPEC)
    #define MOTOR_NAME            "Base-2 (240 deg)"
    #define MOTOR_CAN_CMD_ID      OMNI_ROBOT_R_PI_COMMAND_2_FRAME_ID
    #define MOTOR_CAN_STATUS_ID   OMNI_ROBOT_MCU_STATUS_2_FRAME_ID
    #define MOTOR_CAN_CMD_T       omni_robot_r_pi_command_2_t
    #define MOTOR_CAN_STATUS_T    omni_robot_mcu_status_2_t
    #define MOTOR_CAN_CMD_UNPACK  omni_robot_r_pi_command_2_unpack
    #define MOTOR_CAN_STATUS_PACK omni_robot_mcu_status_2_pack
    #define MOTOR_CAN_STATUS_LENGTH OMNI_ROBOT_MCU_STATUS_2_LENGTH

#elif MOTOR_SELECTION == MOTOR_JOINT_3
    #define MOTOR_ID              3
    #define MOTOR_SPEC            ((hiwonder_servo_spec_t)HTD_45H_SPEC)
    #define MOTOR_NAME            "Joint-3 (240 deg)"
    #define MOTOR_CAN_CMD_ID      OMNI_ROBOT_R_PI_COMMAND_3_FRAME_ID
    #define MOTOR_CAN_STATUS_ID   OMNI_ROBOT_MCU_STATUS_3_FRAME_ID
    #define MOTOR_CAN_CMD_T       omni_robot_r_pi_command_3_t
    #define MOTOR_CAN_STATUS_T    omni_robot_mcu_status_3_t
    #define MOTOR_CAN_CMD_UNPACK  omni_robot_r_pi_command_3_unpack
    #define MOTOR_CAN_STATUS_PACK omni_robot_mcu_status_3_pack
    #define MOTOR_CAN_STATUS_LENGTH OMNI_ROBOT_MCU_STATUS_3_LENGTH

#elif MOTOR_SELECTION == MOTOR_JOINT_4
    #define MOTOR_ID              4
    #define MOTOR_SPEC            ((hiwonder_servo_spec_t)HTD_35H_SPEC)
    #define MOTOR_NAME            "Joint-4 (240 deg)"
    #define MOTOR_CAN_CMD_ID      OMNI_ROBOT_R_PI_COMMAND_4_FRAME_ID
    #define MOTOR_CAN_STATUS_ID   OMNI_ROBOT_MCU_STATUS_4_FRAME_ID
    #define MOTOR_CAN_CMD_T       omni_robot_r_pi_command_4_t
    #define MOTOR_CAN_STATUS_T    omni_robot_mcu_status_4_t
    #define MOTOR_CAN_CMD_UNPACK  omni_robot_r_pi_command_4_unpack
    #define MOTOR_CAN_STATUS_PACK omni_robot_mcu_status_4_pack
    #define MOTOR_CAN_STATUS_LENGTH OMNI_ROBOT_MCU_STATUS_4_LENGTH

#else
    #error "motorSelection.h: MOTOR_SELECTION is not set to a known motor. \
Uncomment one of the MOTOR_JOINT_x defines."
#endif
