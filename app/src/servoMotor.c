/**
  *****************************************************************************
  * @file    servoMotor.c
  * @author  Jacky Lim
  * @brief   Servo motor task – single MCU, daisy-chain bus.
  *
  *          Up to MOTOR_MAX servos share one half-duplex UART bus.  Each
  *          servo is distinguished by its unique servo bus ID (1..MOTOR_MAX).
  *
  *          Boot behaviour
  *          ──────────────
  *          1. Initialise all MOTOR_MAX servo handles (no bus traffic).
  *          2. Ping every ID 1..MOTOR_MAX; mark responding ones present.
  *          3. Enable torque only on present servos.
  *
  *          Runtime reconfiguration
  *          ───────────────────────
  *          RPi sends RPi_Reconfig (0x300) with an explicit list of up to 6
  *          servo IDs.  canRxTask() stores them in g_can_reconfig (pending=true).
  *          servoMotorTask detects the pending flag, calls motor_scan() with the
  *          new ID list, then continues normal operation.
  *
  *          Commands arrive per-slot over CAN (RPi_Command_N) via can_driver.c,
  *          stored in g_can_rx[i].  Status is published at a fixed rate.
  ******************************************************************************
  */

#include "motorSelection.h"
#include "can_driver.h"
#include "hiwonder_bus_servo.h"
#include "debug.h"
#include "logger.h"
#include "bsp.h"

/* CAN status publish rate (applies to every present motor) */
#define SERVO_STATUS_PERIOD_MS   100u

/* ── Runtime presence state (extern declared in motorSelection.h) ── */
volatile uint8_t g_active_motor_count      = 0u;
volatile bool    g_servo_present[MOTOR_MAX] = { false };

/* ── Shared UART bus mutex – passed to every servo handle ────────── */
static SemaphoreHandle_t servo_bus_mutex = NULL;

/* ── Servo handle array – indexed 0..MOTOR_MAX-1 ────────────────── */
hiwonder_servo_t servo[MOTOR_MAX];

// #define ENABLE_PUBLISH_STATUS
#define ENABLE_MOTOR_TEST   // Temporary: test motor comms without CAN

/* ========================================================================== */
/* motor_scan()                                                                */
/*                                                                             */
/* Disable all currently present servos, then ping each requested ID.         */
/* Responding IDs are re-enabled and marked present.                           */
/*                                                                             */
/* @param requested_ids  Array of servo bus IDs to probe (0 = skip).          */
/* @param count          Number of entries in requested_ids[].                 */
/* ========================================================================== */
static void motor_scan(const uint8_t *requested_ids, uint8_t count)
{
    /* ── 1. Disable torque on all currently present slots ────────── */
    for (uint8_t i = 0; i < MOTOR_MAX; i++)
    {
        if (g_servo_present[i]) {
            HWSERVO_EnableTorque(&servo[i], false);
        }
        g_servo_present[i] = false;
    }
    g_active_motor_count = 0u;

    /* ── 2. Probe each requested ID ──────────────────────────────── */
    for (uint8_t k = 0; k < count; k++)
    {
        uint8_t id = requested_ids[k];
        if (id == 0u || id > MOTOR_MAX) {
            continue;  /* gap / unused slot */
        }

        uint8_t idx = (uint8_t)(id - 1u);  /* 0-based index */

        /* Reinitialise the handle with this servo's spec in case the
         * slot was previously assigned a different motor type. */
        HWSERVO_Init(&servo[idx],
                     SERVO_UART,
                     SERVO_DIR_GPIO_Port, SERVO_DIR_Pin,
                     true,
                     servo_bus_mutex,
                     id,
                     g_motor_configs[idx].spec);

        if (HWSERVO_Ping(&servo[idx]) == HWSERVO_OK)
        {
            g_servo_present[idx] = true;
            g_active_motor_count++;
            HWSERVO_EnableTorque(&servo[idx], true);
            LOG_DEBUG("motor_scan: found %s (bus ID=%u, slot=%u)\r\n",
                      g_motor_configs[idx].name, id, idx);
        }
        else
        {
            LOG_WARNING("motor_scan: no response from bus ID=%u (%s)\r\n",
                        id, g_motor_configs[idx].name);
        }
    }

    LOG_DEBUG("motor_scan: %u motor(s) present\r\n", g_active_motor_count);
}

/* ========================================================================== */

void servoMotorTask(void const *argument)
{
    (void)argument;

    /* ── Create the shared bus mutex ────────────────────────────── */
    servo_bus_mutex = xSemaphoreCreateMutex();
    configASSERT(servo_bus_mutex != NULL);

    /* ── Initialise all MOTOR_MAX handles (no bus traffic yet) ───── */
    for (uint8_t i = 0; i < MOTOR_MAX; i++)
    {
        HWSERVO_Init(&servo[i],
                     SERVO_UART,
                     SERVO_DIR_GPIO_Port, SERVO_DIR_Pin,
                     true,
                     servo_bus_mutex,
                     g_motor_configs[i].servo_id,
                     g_motor_configs[i].spec);
    }

    /* ── Boot scan: probe all IDs 1..MOTOR_MAX ───────────────────── */
    static const uint8_t boot_ids[MOTOR_MAX] = { 1u, 2u, 3u, 4u, 5u, 6u };
    motor_scan(boot_ids, MOTOR_MAX);

    LOG_DEBUG("Servo task started: %u motor(s) discovered\r\n",
              g_active_motor_count);

    TickType_t xLastStatusTick = xTaskGetTickCount();

#ifdef ENABLE_MOTOR_TEST
    /* Per-motor test state: current target angle */
    static float test_deg[MOTOR_MAX];
    for (uint8_t i = 0; i < MOTOR_MAX; i++) {
        test_deg[i] = 0.0f;
    }
    uint8_t test_motor = 0;  /* round-robin through present motors */
#endif

    while (1)
    {
        /* ── Check for pending reconfiguration ─────────────────────── */
        if (can_rx_mutex != NULL &&
            xSemaphoreTake(can_rx_mutex, pdMS_TO_TICKS(2)) == pdTRUE)
        {
            bool do_reconfig = g_can_reconfig.pending;
            uint8_t reconfig_ids[MOTOR_MAX];
            if (do_reconfig) {
                for (uint8_t i = 0; i < MOTOR_MAX; i++) {
                    reconfig_ids[i] = g_can_reconfig.servo_ids[i];
                }
                g_can_reconfig.pending = false;
            }
            xSemaphoreGive(can_rx_mutex);

            if (do_reconfig) {
                LOG_DEBUG("Reconfig requested – rescanning bus...\r\n");
                motor_scan(reconfig_ids, MOTOR_MAX);
            }
        }

#ifdef ENABLE_MOTOR_TEST
        /* ── Temporary motor comms test (no CAN required) ──────────── */
        /* Skip non-present slots */
        {
            uint8_t tries = 0;
            while (!g_servo_present[test_motor] && tries < MOTOR_MAX) {
                test_motor = (test_motor + 1u) % MOTOR_MAX;
                tries++;
            }
        }

        if (g_servo_present[test_motor])
        {
            const motor_config_t *cfg = &g_motor_configs[test_motor];
            hiwonder_servo_t     *s   = &servo[test_motor];

            HWSERVO_MoveToAngle(s, test_deg[test_motor], 1000);

            vTaskDelay(pdMS_TO_TICKS(1500));

            int16_t actual_raw = 0;
            if (HWSERVO_ReadPos_Raw(s, &actual_raw) == HWSERVO_OK) {
                LOG_DEBUG("%s: TEST read <- %d raw\r\n", cfg->name, actual_raw);
            } else {
                LOG_DEBUG("%s: TEST readback FAILED\r\n", cfg->name);
            }

            test_deg[test_motor] += 20.0f;
            if (test_deg[test_motor] > s->spec.deg_max) {
                test_deg[test_motor] = s->spec.deg_min;
            }
        }

        test_motor = (test_motor + 1u) % MOTOR_MAX;

#else /* Normal CAN-driven operation */

        /* ── Process incoming CAN commands (only for present motors) ─ */
        for (uint8_t i = 0; i < MOTOR_MAX; i++)
        {
            if (!g_servo_present[i]) {
                continue;
            }

            const motor_config_t *cfg = &g_motor_configs[i];
            hiwonder_servo_t     *s   = &servo[i];

            if (can_rx_mutex == NULL) {
                continue;
            }

            if (xSemaphoreTake(can_rx_mutex, pdMS_TO_TICKS(2)) != pdTRUE) {
                continue;
            }

            if (!g_can_rx[i].fresh) {
                xSemaphoreGive(can_rx_mutex);
                continue;
            }

            /* Consume the command */
            motor_cmd_t cmd   = g_can_rx[i].cmd;
            g_can_rx[i].fresh = false;
            xSemaphoreGive(can_rx_mutex);

            if (cmd.stop_cmd)
            {
                int16_t raw = 0;
                if (HWSERVO_ReadPos_Raw(s, &raw) == HWSERVO_OK) {
                    HWSERVO_MoveTimeWrite_Raw(s, (uint16_t)raw, 0);
                }
                LOG_DEBUG("%s: CAN stop\r\n", cfg->name);
            }
            else
            {
                HWSERVO_EnableTorque(s, cmd.torque_enable != 0u);

                if (cmd.torque_enable)
                {
                    float deg = (float)cmd.target_angle_deg * 0.01f;
                    HWSERVO_MoveToAngle(s, deg, cmd.move_duration_ms);
                }
            }
        } /* end for each motor slot */

#ifdef ENABLE_PUBLISH_STATUS
        /* ── Publish MCU_Status at fixed rate (present motors only) ── */
        if ((xTaskGetTickCount() - xLastStatusTick) >=
            pdMS_TO_TICKS(SERVO_STATUS_PERIOD_MS))
        {
            xLastStatusTick = xTaskGetTickCount();

            for (uint8_t i = 0; i < MOTOR_MAX; i++)
            {
                if (!g_servo_present[i]) {
                    continue;
                }

                hiwonder_servo_t *s = &servo[i];
                motor_status_t status = { 0 };

                float deg = 0.0f;
                if (HWSERVO_ReadAngle_deg(s, &deg) == HWSERVO_OK) {
                    status.joint_angle_deg = (uint16_t)(deg * 100.0f + 0.5f);
                }

                uint16_t mv = 0;
                if (HWSERVO_ReadVin_mV(s, &mv) == HWSERVO_OK) {
                    status.joint_voltage_m_v = mv;
                }

                uint8_t temp = 0;
                if (HWSERVO_ReadTemp_C(s, &temp) == HWSERVO_OK) {
                    status.joint_temp_c = temp;
                }

                status.torque_enabled = 1u;
                CAN_SendMcuStatus(i, &status);
            }
        }
#endif /* ENABLE_PUBLISH_STATUS */

        vTaskDelay(pdMS_TO_TICKS(25));
#endif /* ENABLE_MOTOR_TEST */
    }
}
