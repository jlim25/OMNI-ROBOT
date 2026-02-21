/**
  *****************************************************************************
  * @file    servoMotor.c
  * @author  Jacky Lim
  * @brief   Task for testing the Hiwonder Bus Servo (HTD45H) implementation.
  ******************************************************************************
  */

#include "htd45h.h"
#include "debug.h"
#include "logger.h"


static htd45h_t servo;
static SemaphoreHandle_t servo_bus_mutex = NULL;

void servoMotorTask(void const *argument)
{
    // Create mutex before initializing the servo driver
    servo_bus_mutex = xSemaphoreCreateMutex();
    configASSERT(servo_bus_mutex != NULL);  // halt if allocation failed

    // init HTD45H device
    HTD45H_Init(&servo, SERVO_UART, SERVO_DIR_GPIO_Port, SERVO_DIR_Pin, true, servo_bus_mutex);

    // Enable torque for servo ID 1
    HTD45H_EnableTorque(&servo, 1, true);

    uint16_t remainingStack = uxTaskGetStackHighWaterMark(NULL); // for debugging stack usage
    LOG_DEBUG("Servo Motor Task Stack High Water Mark: %u\r\n", remainingStack);

    while (1)
    {
        HTD45H_MoveTimeWrite(&servo, 1, 500, 1000);
        vTaskDelay(pdMS_TO_TICKS(1500));

        int16_t pos;
        if (HTD45H_ReadPos(&servo, 1, &pos) == HTD_OK)
        {
            LOG_DEBUG("POS: %d\r\n", pos);
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
