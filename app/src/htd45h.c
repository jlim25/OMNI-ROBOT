/**
  *****************************************************************************
  * @file    htd45h.c
  * @author  Jacky Lim
  * @brief   HTD45H implementation for controlling Hiwonder Bus Servos.
  ******************************************************************************
  */

#include "htd45h.h"
#include "string.h"
#include "debug.h"

#define HTD_MAX_FRAME   32  // plenty for these commands

/* ── Helper functions ─────────────────────────────────────────── */

static inline void htd_set_dir_tx(const htd45h_t *dev, bool tx)
{
    GPIO_PinState st;

    if (dev->dir_tx_high) {
        st = tx ? GPIO_PIN_SET : GPIO_PIN_RESET;
    } else {
        st = tx ? GPIO_PIN_RESET : GPIO_PIN_SET;
    }

    HAL_GPIO_WritePin(dev->dir_port, dev->dir_pin, st);
}

static uint8_t htd_checksum(uint8_t id, uint8_t len, uint8_t cmd,
                            const uint8_t *params, uint8_t nparams)
{
    // Checksum = ~(ID + Length + Cmd + Prm1 + ... + PrmN)  (low byte)
    uint16_t sum = 0;
    sum += id;
    sum += len;
    sum += cmd;
    for (uint8_t i = 0; i < nparams; i++) {
        sum += params[i];
    }
    // Note if the checksum exceeds 255, then take the lowest byte 
    return (uint8_t)(~(sum & 0xFF));
}

static htd_status_t htd_lock(const htd45h_t *dev)
{
    if (dev->bus_mutex == NULL) return HTD_OK;
    return (xSemaphoreTake(dev->bus_mutex, portMAX_DELAY) == pdTRUE)
           ? HTD_OK : HTD_ERR_BUSY;
}

static void htd_unlock(const htd45h_t *dev)
{
    if (dev->bus_mutex != NULL) {
        xSemaphoreGive(dev->bus_mutex);
    }
}

static htd_status_t htd_tx_frame(htd45h_t *dev, const uint8_t *buf, uint16_t len)
{
    // Set direction to TX before starting transmission
    htd_set_dir_tx(dev, true);

    // Small settle time for direction-controlled buffer (optional but helps)
    // For 115200 baud, even a few us is enough; 1 tick is overkill.
    for (volatile int i = 0; i < 200; i++) { __NOP(); }

    if (HAL_UART_Transmit(dev->huart, (uint8_t*)buf, len, dev->tx_timeout_ms) != HAL_OK) {
        return HTD_ERR_TIMEOUT;
    }

    // Ensure last bit is out before switching to RX
    // (HAL_UART_Transmit should block until done, but TC wait makes it robust)
    while (__HAL_UART_GET_FLAG(dev->huart, UART_FLAG_TC) == RESET) { } // TODO: double check the value of flag 

    return HTD_OK;
}

// Receives one byte with timeout. Caller should call this repeatedly to receive a full frame.
static htd_status_t htd_rx_byte(htd45h_t *dev, uint8_t *b, uint32_t timeout_ms)
{
    if (HAL_UART_Receive(dev->huart, b, 1, timeout_ms) != HAL_OK) {
        return HTD_ERR_TIMEOUT;
    }
    return HTD_OK;
}

// Receives a full frame into out_buf. Returns total frame length in *out_len.
// Frame format: 55 55 ID LEN CMD ... CHECKSUM
// LEN includes itself, so bytes from LEN to CHECKSUM inclusive == LEN bytes.
// Total bytes from first 0x55 to CHECKSUM inclusive == 2 (hdr) + 1 (ID) + LEN.
static htd_status_t htd_rx_frame(htd45h_t *dev, uint8_t *out_buf, uint16_t out_max, uint16_t *out_len)
{
    htd_set_dir_tx(dev, false); // set direction to RX before receiving
    for (volatile int i = 0; i < 200; i++) { __NOP(); } // delay for direction change to take effect (optional but helps)
    // TODO: check if the delay above is needed or not

    // Hunt for header 0x55 0x55
    uint8_t b = 0;
    uint8_t prev = 0;
    uint32_t start_timeout = dev->rx_timeout_ms;

    while (1) {
        htd_status_t st = htd_rx_byte(dev, &b, start_timeout);
        if (st != HTD_OK) return st;

        if (prev == HTD_FRAME_HEADER && b == HTD_FRAME_HEADER) {
            break;
        }
        prev = b;
        start_timeout = dev->rx_timeout_ms;
    }

    // We have "55 55"
    uint16_t idx = 0;
    out_buf[idx++] = HTD_FRAME_HEADER;
    out_buf[idx++] = HTD_FRAME_HEADER;

    uint8_t id = 0, len = 0;
    if (htd_rx_byte(dev, &id, dev->rx_timeout_ms) != HTD_OK) return HTD_ERR_TIMEOUT;
    if (htd_rx_byte(dev, &len, dev->rx_timeout_ms) != HTD_OK) return HTD_ERR_TIMEOUT;

    out_buf[idx++] = id;
    out_buf[idx++] = len;

    // Remaining bytes after LEN field: (LEN - 1) = CMD + PARAMS + CHECKSUM
    if (len < 3) return HTD_ERR_FRAME; // minimum: LEN, CMD, CHECKSUM

    uint16_t remain = (uint16_t)(len - 1);
    uint16_t total_needed = (uint16_t)(2 + 1 + len); // hdr(2) + id(1) + len bytes (from LEN..checksum)

    if (total_needed > out_max) return HTD_ERR_FRAME;

    for (uint16_t i = 0; i < remain; i++) {
        if (htd_rx_byte(dev, &out_buf[idx++], dev->rx_timeout_ms) != HTD_OK) return HTD_ERR_TIMEOUT;
    }

    *out_len = idx;
    return HTD_OK;
}

static htd_status_t htd_build_frame(uint8_t id, uint8_t cmd,
                                    const uint8_t *params, uint8_t nparams,
                                    uint8_t *out, uint16_t out_max, uint16_t *out_len)
{
    // LEN includes itself: LEN = 3 + Nparams  (LEN + CMD + ... + CHECKSUM)【protocol】
    uint8_t len = (uint8_t)(3 + nparams);

    // Total bytes = 2 (hdr) + 1 (id) + len
    uint16_t total = (uint16_t)(2 + 1 + len);
    if (total > out_max) return HTD_ERR_PARAM;

    uint16_t i = 0;
    out[i++] = 0x55;
    out[i++] = 0x55;
    out[i++] = id;
    out[i++] = len;
    out[i++] = cmd;

    for (uint8_t p = 0; p < nparams; p++) {
        out[i++] = params[p];
    }

    out[i++] = htd_checksum(id, len, cmd, params, nparams);

    *out_len = i;
    return HTD_OK;
}

static htd_status_t htd_send_and_optional_read(htd45h_t *dev,
                                               uint8_t id, uint8_t cmd,
                                               const uint8_t *params, uint8_t nparams,
                                               bool expect_reply,
                                               uint8_t *rx_buf, uint16_t rx_max, uint16_t *rx_len)
{
    uint8_t tx[HTD_MAX_FRAME];
    uint16_t tx_len = 0;

    htd_status_t st = htd_build_frame(id, cmd, params, nparams, tx, sizeof(tx), &tx_len);
    if (st != HTD_OK) return st;

    st = htd_tx_frame(dev, tx, tx_len);
    if (st != HTD_OK) return st;

    if (!expect_reply) {
        return HTD_OK;
    }

    return htd_rx_frame(dev, rx_buf, rx_max, rx_len);
}

/* ── Public API ─────────────────────────────────────────── */
htd_status_t HTD45H_Init(htd45h_t *dev,
                         UART_HandleTypeDef *huart,
                         GPIO_TypeDef *dir_port, uint16_t dir_pin,
                         bool dir_tx_high,
                         SemaphoreHandle_t bus_mutex)
{
    if (!dev || !huart || !dir_port) return HTD_ERR_PARAM;

    memset(dev, 0, sizeof(*dev));
    dev->huart = huart;
    dev->dir_port = dir_port;
    dev->dir_pin = dir_pin;
    dev->dir_tx_high = dir_tx_high;
    dev->bus_mutex = bus_mutex;

    dev->tx_timeout_ms = 20;
    dev->rx_timeout_ms = 50;

    // Default to RX direction on init
    htd_set_dir_tx(dev, false);
    return HTD_OK;
}

// ===== High-level commands =====

// Move to position (0-1000) in given time (ms). Time=0 means use default time
htd_status_t HTD45H_MoveTimeWrite(htd45h_t *dev, uint8_t id,
                                  uint16_t pos_0_1000, uint16_t time_ms)
{
    if (!dev) return HTD_ERR_PARAM;
    if (pos_0_1000 > 1000) return HTD_ERR_PARAM;
    if (time_ms > 30000)   return HTD_ERR_PARAM;

    // Params: posL, posH, timeL, timeH
    uint8_t prm[4];
    prm[0] = (uint8_t)(pos_0_1000 & 0xFF);
    prm[1] = (uint8_t)((pos_0_1000 >> 8) & 0xFF);
    prm[2] = (uint8_t)(time_ms & 0xFF);
    prm[3] = (uint8_t)((time_ms >> 8) & 0xFF);

    htd_status_t st = htd_lock(dev);
    if (st != HTD_OK) return st;

    // Write command: no response expected
    st = htd_send_and_optional_read(dev, id, HTD_CMD_SERVO_MOVE_TIME_WRITE,
                                    prm, sizeof(prm),
                                    false, NULL, 0, NULL);

    htd_unlock(dev);
    return st;
}

// htd_status_t HTD45H_SetTorque(htd45h_t *dev, uint8_t id, bool enable)
// {
//     if (!dev) return HTD_ERR_PARAM;

//     // SERVO_LOAD_OR_UNLOAD_WRITE (31) length=4 => 1 parameter: 0=unload, 1=load
//     uint8_t prm[1];
//     prm[0] = enable ? 1 : 0;

//     htd_status_t st = htd_lock(dev);
//     if (st != HTD_OK) return st;

//     st = htd_send_and_optional_read(dev, id, HTD_CMD_SERVO_LOAD_OR_UNLOAD_WRITE,
//                                     prm, sizeof(prm),
//                                     false, NULL, 0, NULL);

//     htd_unlock(dev);
//     return st;
// }

static htd_status_t parse_and_check_reply(const uint8_t *rx, uint16_t rx_len,
                                          uint8_t expect_id, uint8_t expect_cmd,
                                          const uint8_t **params_out, uint8_t *nparams_out)
{
    if (rx_len < 2 + 1 + 3) return HTD_ERR_FRAME; // minimal frame
    if (rx[0] != 0x55 || rx[1] != 0x55) return HTD_ERR_FRAME;

    uint8_t id  = rx[2];
    uint8_t len = rx[3];
    uint8_t cmd = rx[4];

    if (id != expect_id) return HTD_ERR_FRAME;
    if (cmd != expect_cmd) return HTD_ERR_FRAME;

    // Total bytes should be 2 + 1 + len
    uint16_t expected_total = (uint16_t)(2 + 1 + len);
    if (rx_len != expected_total) return HTD_ERR_FRAME;

    // params length = len - 3  (LEN includes itself + CMD + CHK)
    uint8_t nparams = (uint8_t)(len - 3);

    const uint8_t *params = &rx[5];
    uint8_t chk = rx[rx_len - 1];

    uint8_t calc = htd_checksum(id, len, cmd, params, nparams);
    if (chk != calc) return HTD_ERR_CHECKSUM;

    *params_out = params;
    *nparams_out = nparams;
    return HTD_OK;
}

htd_status_t HTD45H_ReadPos(htd45h_t *dev, uint8_t id, int16_t *pos_out)
{
    if (!dev || !pos_out) return HTD_ERR_PARAM;

    uint8_t rx[HTD_MAX_FRAME];
    uint16_t rx_len = 0;

    htd_status_t st = htd_lock(dev);
    if (st != HTD_OK) return st;

    st = htd_send_and_optional_read(dev, id, HTD_CMD_SERVO_POS_READ,
                                    NULL, 0,
                                    true, rx, sizeof(rx), &rx_len);

    htd_unlock(dev);
    if (st != HTD_OK) return st;

    const uint8_t *params = NULL;
    uint8_t nparams = 0;
    st = parse_and_check_reply(rx, rx_len, id, HTD_CMD_SERVO_POS_READ, &params, &nparams);
    if (st != HTD_OK) return st;

    // POS_READ returns 2 bytes position (signed short)
    if (nparams < 2) return HTD_ERR_FRAME;
    int16_t pos = (int16_t)((int16_t)params[0] | ((int16_t)params[1] << 8));
    *pos_out = pos;
    return HTD_OK;
}

htd_status_t HTD45H_ReadVin_mV(htd45h_t *dev, uint8_t id, uint16_t *mv_out)
{
    if (!dev || !mv_out) return HTD_ERR_PARAM;

    uint8_t rx[HTD_MAX_FRAME];
    uint16_t rx_len = 0;

    htd_status_t st = htd_lock(dev);
    if (st != HTD_OK) return st;

    st = htd_send_and_optional_read(dev, id, HTD_CMD_SERVO_VIN_READ,
                                    NULL, 0,
                                    true, rx, sizeof(rx), &rx_len);

    htd_unlock(dev);
    if (st != HTD_OK) return st;

    const uint8_t *params = NULL;
    uint8_t nparams = 0;
    st = parse_and_check_reply(rx, rx_len, id, HTD_CMD_SERVO_VIN_READ, &params, &nparams);
    if (st != HTD_OK) return st;

    // VIN_READ returns 2 bytes voltage in mV
    if (nparams < 2) return HTD_ERR_FRAME;
    *mv_out = (uint16_t)(params[0] | ((uint16_t)params[1] << 8));
    return HTD_OK;
}

htd_status_t HTD45H_ReadTemp_C(htd45h_t *dev, uint8_t id, uint8_t *tempC_out)
{
    if (!dev || !tempC_out) return HTD_ERR_PARAM;

    uint8_t rx[HTD_MAX_FRAME];
    uint16_t rx_len = 0;

    htd_status_t st = htd_lock(dev);
    if (st != HTD_OK) return st;

    st = htd_send_and_optional_read(dev, id, HTD_CMD_SERVO_TEMP_READ,
                                    NULL, 0,
                                    true, rx, sizeof(rx), &rx_len);

    htd_unlock(dev);
    if (st != HTD_OK) return st;

    const uint8_t *params = NULL;
    uint8_t nparams = 0;
    st = parse_and_check_reply(rx, rx_len, id, HTD_CMD_SERVO_TEMP_READ, &params, &nparams);
    if (st != HTD_OK) return st;

    // TEMP_READ returns 1 byte temperature
    if (nparams < 1) return HTD_ERR_FRAME;
    *tempC_out = params[0];
    return HTD_OK;
}