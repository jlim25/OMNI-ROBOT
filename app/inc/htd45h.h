#pragma once

#include "debug.h"

#ifdef __cplusplus
extern "C" {
#endif

// ===== Commands (from Hiwonder Bus Servo Protocol) =====
#define HTD_CMD_SERVO_MOVE_TIME_WRITE       0x01
#define HTD_CMD_SERVO_POS_READ              0x1C  // 28
#define HTD_CMD_SERVO_TEMP_READ             0x1A  // 26
#define HTD_CMD_SERVO_VIN_READ              0x1B  // 27
#define HTD_CMD_SERVO_LOAD_OR_UNLOAD_WRITE  0x1F  // 31

#define HTD_BROADCAST_ID                     0xFE
#define HTD_FRAME_HEADER                     0x55

typedef enum {
    HTD_OK = 0,
    HTD_ERR_TIMEOUT,
    HTD_ERR_CHECKSUM,
    HTD_ERR_FRAME,
    HTD_ERR_BUSY,
    HTD_ERR_PARAM
} htd_status_t;

typedef struct {
    UART_HandleTypeDef *huart;

    GPIO_TypeDef *dir_port;
    uint16_t      dir_pin;

    // Set this true if DIR=1 means TX (MCU->Servo). If opposite, set false.
    bool dir_tx_high;

    // FreeRTOS protection
    SemaphoreHandle_t bus_mutex;

    // Tuning
    uint32_t tx_timeout_ms;
    uint32_t rx_timeout_ms;
} htd45h_t;

htd_status_t HTD45H_Init(htd45h_t *dev,
                         UART_HandleTypeDef *huart,
                         GPIO_TypeDef *dir_port, uint16_t dir_pin,
                         bool dir_tx_high,
                         SemaphoreHandle_t bus_mutex);

htd_status_t HTD45H_MoveTimeWrite(htd45h_t *dev, uint8_t id,
                                  uint16_t pos_0_1000, uint16_t time_ms);

htd_status_t HTD45H_SetTorque(htd45h_t *dev, uint8_t id, bool enable);

htd_status_t HTD45H_ReadPos(htd45h_t *dev, uint8_t id, int16_t *pos_out);
htd_status_t HTD45H_ReadVin_mV(htd45h_t *dev, uint8_t id, uint16_t *mv_out);
htd_status_t HTD45H_ReadTemp_C(htd45h_t *dev, uint8_t id, uint8_t *tempC_out);

#ifdef __cplusplus
}
#endif