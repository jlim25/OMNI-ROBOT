/**
  *****************************************************************************
  * @file    servoMotor.c
  * @author  Jacky Lim
  * @brief   Servo motor task.
  *
  *          The motor connected to this MCU is chosen at compile-time via
  *          motorSelection.h (edit that file to change the joint assignment).
  ******************************************************************************
  */

#include "motorSelection.h"  // defines MOTOR_ID, MOTOR_SPEC, MOTOR_NAME
#include "debug.h"
#include "logger.h"

static hiwonder_servo_t servo;
static SemaphoreHandle_t servo_bus_mutex = NULL;

void servoMotorTask(void const *argument)
{
    // Create mutex before initializing the servo driver
    servo_bus_mutex = xSemaphoreCreateMutex();
    configASSERT(servo_bus_mutex != NULL);

    HWSERVO_Init(&servo,
                 SERVO_UART,
                 SERVO_DIR_GPIO_Port, SERVO_DIR_Pin,
                 true,
                 servo_bus_mutex,
                 MOTOR_ID,
                 MOTOR_SPEC);

    LOG_DEBUG("Servo task started: " MOTOR_NAME " (ID=%u)\r\n", MOTOR_ID);

    HWSERVO_EnableTorque(&servo, true);

    uint16_t remainingStack = uxTaskGetStackHighWaterMark(NULL);

    while (1)
    {
        // Example: move to mid-range angle over 1 second
        float mid_deg = (servo.spec.deg_min + servo.spec.deg_max) / 2.0f;
        HWSERVO_MoveToAngle(&servo, mid_deg, 1000);
        vTaskDelay(pdMS_TO_TICKS(1500));

        float angle = 0.0f;
        if (HWSERVO_ReadAngle_deg(&servo, &angle) == HWSERVO_OK)
        {
            LOG_DEBUG(MOTOR_NAME " angle: %.1f deg\r\n", angle);
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
