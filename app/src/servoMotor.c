/**
  *****************************************************************************
  * @file    servoMotor.c
  * @author  Jacky Lim
  * @brief   Servo motor task.
  *
  *          The motor connected to this MCU is chosen at compile-time via
  *          motorSelection.h (edit that file to change the joint assignment).
  *
  *          Commands arrive over CAN (RPi_Command) via can_driver.c.
  *          Status (angle, voltage, temp) is published over CAN (MCU_Status)
  *          at a fixed rate.
  ******************************************************************************
  */

#include "motorSelection.h"  // MOTOR_ID, MOTOR_SPEC, MOTOR_NAME
#include "can_driver.h"
#include "debug.h"
#include "logger.h"

/* CAN status publish rate */
#define SERVO_STATUS_PERIOD_MS   100u

/* Non-static so cli_commands.c can reach this handle via extern */
hiwonder_servo_t servo;
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

    TickType_t xLastStatusTick = xTaskGetTickCount();

    while (1)
    {
        /* ── Process incoming CAN command ───────────────────────── */
        if (can_rx_mutex != NULL &&
            xSemaphoreTake(can_rx_mutex, pdMS_TO_TICKS(2)) == pdTRUE)
        {
            if (g_can_rx.fresh)
            {
                struct MOTOR_CAN_CMD_T cmd = g_can_rx.cmd;
                g_can_rx.fresh = false;
                xSemaphoreGive(can_rx_mutex);

                if (cmd.stop_cmd)
                {
                    /* Stop: command servo to hold its current position */
                    int16_t raw = 0;
                    if (HWSERVO_ReadPos_Raw(&servo, &raw) == HWSERVO_OK) {
                        HWSERVO_MoveTimeWrite_Raw(&servo, (uint16_t)raw, 0);
                    }
                    LOG_DEBUG(MOTOR_NAME ": CAN stop\r\n");
                }
                else
                {
                    /* Apply torque enable first */
                    HWSERVO_EnableTorque(&servo, cmd.torque_enable != 0u);

                    if (cmd.torque_enable)
                    {
                        /* target_angle_deg raw value, scale=0.01 → degrees */
                        float deg = (float)cmd.target_angle_deg * 0.01f;
                        HWSERVO_MoveToAngle(&servo, deg, cmd.move_duration_ms);
                        LOG_DEBUG(MOTOR_NAME ": CAN move %.1f deg, %u ms\r\n",
                                  deg, cmd.move_duration_ms);
                    }
                }
            }
            else
            {
                xSemaphoreGive(can_rx_mutex);
            }
        }
#ifdef ENABLE_PUBLISH_STATUS
        /* ── Publish MCU_Status over CAN at fixed rate ──────────── */
        if ((xTaskGetTickCount() - xLastStatusTick) >=
            pdMS_TO_TICKS(SERVO_STATUS_PERIOD_MS))
        {
            xLastStatusTick = xTaskGetTickCount();

            struct MOTOR_CAN_STATUS_T status = { 0 };

            float deg = 0.0f;
            if (HWSERVO_ReadAngle_deg(&servo, &deg) == HWSERVO_OK) {
                /* scale back to raw CAN value: deg / 0.01 */
                status.joint_angle_deg = (uint16_t)(deg * 100.0f + 0.5f);
            }

            uint16_t mv = 0;
            if (HWSERVO_ReadVin_mV(&servo, &mv) == HWSERVO_OK) {
                status.joint_voltage_m_v = mv;
            }

            uint8_t temp = 0;
            if (HWSERVO_ReadTemp_C(&servo, &temp) == HWSERVO_OK) {
                status.joint_temp_c = temp;
            }

            status.torque_enabled = 1u; // reflect actual state if tracked

            CAN_SendMcuStatus(&status);
        }
#endif /* ENABLE_PUBLISH_STATUS */
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
