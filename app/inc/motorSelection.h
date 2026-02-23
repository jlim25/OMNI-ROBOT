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
    #define MOTOR_ID    1
    #define MOTOR_SPEC  ((hiwonder_servo_spec_t)HTD_85H_SPEC)
    #define MOTOR_NAME  "Base-1 (240 deg)"

#elif MOTOR_SELECTION == MOTOR_JOINT_2
    #define MOTOR_ID    2
    #define MOTOR_SPEC  ((hiwonder_servo_spec_t)HTD_85H_SPEC)
    #define MOTOR_NAME  "Base-2 (240 deg)"

#elif MOTOR_SELECTION == MOTOR_JOINT_3
    #define MOTOR_ID    3
    #define MOTOR_SPEC  ((hiwonder_servo_spec_t)HTD_45H_SPEC)
    #define MOTOR_NAME  "Joint-3 (240 deg)"

#elif MOTOR_SELECTION == MOTOR_JOINT_4
    #define MOTOR_ID    4
    #define MOTOR_SPEC  ((hiwonder_servo_spec_t)HTD_35H_SPEC)
    #define MOTOR_NAME  "Joint-4 (240 deg)"

#else
    #error "motorSelection.h: MOTOR_SELECTION is not set to a known motor. \
Uncomment one of the MOTOR_JOINT_x defines."
#endif
