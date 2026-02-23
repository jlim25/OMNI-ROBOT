#pragma once

/**
  *****************************************************************************
  * @file    can_driver.h
  * @brief   CAN driver: RX/TX tasks and shared application data.
  *
  *  Architecture
  *  ------------
  *  canRxTask  – woken by ISR via direct task notification; unpacks frames
  *               and updates g_can_rx under a mutex.
  *  canTxTask  – blocks on canTxQueue; packs and sends frames via HAL.
  *
  *  The ISR callback (HAL_CAN_RxFifo0MsgPendingCallback) is implemented
  *  in can_driver.c and notifies canRxTask directly.
  *****************************************************************************
  */

#include "debug.h"
#include "motorSelection.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ── TX queue depth ─────────────────────────────────────────────── */
#define CAN_TX_QUEUE_DEPTH   8

/* ── Raw CAN frame container used for the TX queue ─────────────── */
typedef struct {
    uint32_t id;          // standard 11-bit CAN ID
    uint8_t  dlc;         // data length code (0–8)
    uint8_t  data[8];
} can_raw_frame_t;

/* ── Shared RX state ────────────────────────────────────────────── */
/**
 * Written by canRxTask, read by any task (e.g. servoMotorTask).
 * Always access under can_rx_mutex.
 */
typedef struct {
    struct MOTOR_CAN_CMD_T cmd;  // latest decoded RPi_Command
    bool                   fresh; // true until consumed by reader
} can_rx_state_t;

extern can_rx_state_t  g_can_rx;
extern SemaphoreHandle_t can_rx_mutex;
extern QueueHandle_t     canTxQueue;

/* ── Helper: enqueue a pre-packed MCU_Status frame for TX ───────── */
/**
 * Call from any task to publish a status update to the RPi.
 * Non-blocking: drops the message silently if the queue is full.
 */
void CAN_SendMcuStatus(const struct MOTOR_CAN_STATUS_T *status);

#ifdef __cplusplus
}
#endif
