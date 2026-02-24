/**
  *****************************************************************************
  * @file    can_driver.c
  * @brief   CAN RX/TX tasks and ISR callback.
  *
  *  Flow
  *  ----
  *  RX:  CAN frame arrives → HAL_CAN_RxFifo0MsgPendingCallback() (ISR)
  *         → vTaskNotifyGiveFromISR() → canRxTask() wakes
  *         → HAL_CAN_GetRxMessage() → omni_robot_r_pi_command_unpack()
  *         → update g_can_rx under can_rx_mutex
  *
  *  TX:  Any task calls CAN_SendMcuStatus() → xQueueSend(canTxQueue)
  *         → canTxTask() wakes → HAL_CAN_AddTxMessage()
  *****************************************************************************
  */

#include "can_driver.h"
#include "can.h"
#include "logger.h"
#include "motorSelection.h"  // MOTOR_CAN_CMD_ID, MOTOR_CAN_STATUS_ID

/* ── Shared RX state ────────────────────────────────────────────── */
can_rx_state_t   g_can_rx     = { 0 };
SemaphoreHandle_t can_rx_mutex = NULL;

/* ── TX queue ───────────────────────────────────────────────────── */
QueueHandle_t canTxQueue = NULL;

/* ── Task handle needed by the ISR to send a notification ──────── */
static TaskHandle_t s_canRxTaskHandle = NULL;

/* ── ISR callback ───────────────────────────────────────────────── */
/**
 * Called by HAL inside CAN_RX0_IRQHandler (stm32f3xx_it.c).
 * Do the absolute minimum here: notify the RX task and yield if needed.
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    (void)hcan;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    /* CAN_IT_RX_FIFO0_MSG_PENDING is level-triggered: it stays asserted while
     * the FIFO is non-empty.  Disable it here so the ISR does not re-fire
     * continuously before canRxTask has a chance to drain the FIFO.
     * canRxTask re-enables it after draining. */
    __HAL_CAN_DISABLE_IT(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

    if (s_canRxTaskHandle != NULL) {
        vTaskNotifyGiveFromISR(s_canRxTaskHandle, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* ── canRxTask ──────────────────────────────────────────────────── */
void canRxTask(void const *argument)
{
    (void)argument;

    /* Store our own handle so the ISR can notify us */
    s_canRxTaskHandle = xTaskGetCurrentTaskHandle();

    /* Create the mutex that guards g_can_rx */
    can_rx_mutex = xSemaphoreCreateMutex();
    configASSERT(can_rx_mutex != NULL);

    CAN_RxHeaderTypeDef rxHeader;
    uint8_t             rxData[8];

    for (;;)
    {
        /* Block indefinitely until the ISR fires */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /* Drain all pending frames */
        while (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) > 0u)
        {
            if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0,
                                     &rxHeader, rxData) != HAL_OK) {
                LOG_ERROR("CAN RX read error\r\n");
                break;
            }

            /* Route by CAN ID */
            if (rxHeader.StdId == MOTOR_CAN_CMD_ID)
            {
                struct MOTOR_CAN_CMD_T decoded;

                if (MOTOR_CAN_CMD_UNPACK(&decoded, rxData,
                                         rxHeader.DLC) >= 0)
                {
                    if (xSemaphoreTake(can_rx_mutex, pdMS_TO_TICKS(5))
                        == pdTRUE)
                    {
                        g_can_rx.cmd   = decoded;
                        g_can_rx.fresh = true;
                        xSemaphoreGive(can_rx_mutex);
                    }
                } else {
                    LOG_WARNING("CAN RX unpack failed for RPi_Command\r\n");
                }
            }
            /* Add more else-if blocks here for other message IDs */
        }

        /* FIFO is now empty – re-enable the interrupt so the next arriving
         * message triggers the ISR again. */
        __HAL_CAN_ENABLE_IT(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    }
}

/* ── canTxTask ──────────────────────────────────────────────────── */
void canTxTask(void const *argument)
{
    (void)argument;

    /* Create the TX queue */
    canTxQueue = xQueueCreate(CAN_TX_QUEUE_DEPTH, sizeof(can_raw_frame_t));
    configASSERT(canTxQueue != NULL);

    can_raw_frame_t  frame;
    CAN_TxHeaderTypeDef txHeader = {
        .IDE                = CAN_ID_STD,
        .RTR                = CAN_RTR_DATA,
        .TransmitGlobalTime = DISABLE,
    };
    uint32_t txMailbox = 0;

    for (;;)
    {
        /* Block until something is queued for TX */
        if (xQueueReceive(canTxQueue, &frame, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        txHeader.StdId = frame.id;
        txHeader.DLC   = frame.dlc;

        /* Wait for a free mailbox (up to 10 ms) before giving up */
        uint32_t deadline = HAL_GetTick() + 10u;
        while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0u) {
            if (HAL_GetTick() >= deadline) {
                LOG_WARNING("CAN TX mailbox timeout, dropping frame 0x%03lX\r\n",
                         (unsigned long)frame.id);
                break;
            }
            taskYIELD();
        }

        if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) > 0u) {
            if (HAL_CAN_AddTxMessage(&hcan, &txHeader,
                                     frame.data, &txMailbox) != HAL_OK) {
                LOG_ERROR("CAN TX failed for id 0x%03lX\r\n",
                          (unsigned long)frame.id);
            }
        }
    }
}

/* ── CAN_SendMcuStatus ──────────────────────────────────────────── */
void CAN_SendMcuStatus(const struct MOTOR_CAN_STATUS_T *status)
{
    can_raw_frame_t frame;
    frame.id  = MOTOR_CAN_STATUS_ID;
    frame.dlc = MOTOR_CAN_STATUS_LENGTH;

    if (MOTOR_CAN_STATUS_PACK(frame.data, status,
                                   sizeof(frame.data)) < 0) {
        LOG_ERROR("CAN_SendMcuStatus: pack failed\r\n");
        return;
    }

    /* Non-blocking: drop if queue is full to avoid stalling the caller */
    if (canTxQueue != NULL) {
        xQueueSend(canTxQueue, &frame, 0);
    }
}
