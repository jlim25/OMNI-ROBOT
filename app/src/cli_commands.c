/**
  *****************************************************************************
  * @file    cli_commands.c
  * @brief   Application CLI command definitions for FreeRTOS-Plus-CLI.
  *
  *          Each command is a static CLI_Command_Definition_t paired with a
  *          callback.  All commands are registered via CLI_RegisterAllCommands().
  *
  *  Adding a new command
  *  --------------------
  *  1. Write a static callback:
  *       static BaseType_t prvMyCmd(char *pcWriteBuffer,
  *                                  size_t xWriteBufferLen,
  *                                  const char *pcCommandString);
  *
  *  2. Declare a static CLI_Command_Definition_t:
  *       static const CLI_Command_Definition_t xMyCmd = {
  *           "mycommand",            // what the user types
  *           "\r\nmycommand <a>:\r\n"
  *           "  Does something with <a>.\r\n\r\n",
  *           prvMyCmd,
  *           1                       // number of parameters (-1 = variable)
  *       };
  *
  *  3. Call FreeRTOS_CLIRegisterCommand(&xMyCmd) inside
  *     CLI_RegisterAllCommands().
  *****************************************************************************
  */

#include "FreeRTOS_CLI.h"
#include "cli_commands.h"
#include "motorSelection.h"
#include "debug.h"
#include "string.h"
#include "stdlib.h"

/* Forward-declare callbacks */
static BaseType_t prvMoveAngleCommand(char *pcWriteBuffer, size_t xWriteBufferLen,
                                      const char *pcCommandString);
static BaseType_t prvReadPosCommand(char *pcWriteBuffer, size_t xWriteBufferLen,
                                    const char *pcCommandString);
static BaseType_t prvTorqueCommand(char *pcWriteBuffer, size_t xWriteBufferLen,
                                   const char *pcCommandString);

/* ── Command definitions ────────────────────────────────────────── */

static const CLI_Command_Definition_t xMoveAngleCommand = {
    "move",
    "\r\nmove <degrees> <time_ms>:\r\n"
    "  Move the servo to <degrees> over <time_ms> milliseconds.\r\n"
    "  Example: move 120.0 1000\r\n\r\n",
    prvMoveAngleCommand,
    2   /* degrees, time_ms */
};

static const CLI_Command_Definition_t xReadPosCommand = {
    "readpos",
    "\r\nreadpos:\r\n"
    "  Read the current servo position (degrees and raw value).\r\n\r\n",
    prvReadPosCommand,
    0
};

static const CLI_Command_Definition_t xTorqueCommand = {
    "torque",
    "\r\ntorque <on|off>:\r\n"
    "  Enable or disable servo torque.\r\n"
    "  Example: torque on\r\n\r\n",
    prvTorqueCommand,
    1   /* on / off */
};

/* ── Registration ───────────────────────────────────────────────── */

void CLI_RegisterAllCommands(void)
{
    FreeRTOS_CLIRegisterCommand(&xMoveAngleCommand);
    FreeRTOS_CLIRegisterCommand(&xReadPosCommand);
    FreeRTOS_CLIRegisterCommand(&xTorqueCommand);
}

/* ── Servo handle (shared with servoMotor.c via extern) ─────────── */
/* Declare this extern so CLI commands can reach the same handle that
 * servoMotorTask() initialised.  The definition lives in servoMotor.c. */
extern hiwonder_servo_t servo;

/* ── Callbacks ──────────────────────────────────────────────────── */
static BaseType_t prvMoveAngleCommand(char *pcWriteBuffer, size_t xWriteBufferLen,
                                      const char *pcCommandString)
{
    BaseType_t xLen1 = 0, xLen2 = 0;

    const char *pcDeg  = FreeRTOS_CLIGetParameter(pcCommandString, 1, &xLen1);
    const char *pcTime = FreeRTOS_CLIGetParameter(pcCommandString, 2, &xLen2);

    if (pcDeg == NULL || pcTime == NULL) {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "Usage: move <degrees> <time_ms>\r\n");
        return pdFALSE;
    }

    float deg     = strtof(pcDeg,  NULL);
    uint16_t t_ms = (uint16_t)strtoul(pcTime, NULL, 10);

    hwservo_status_t st = HWSERVO_MoveToAngle(&servo, deg, t_ms);

    if (st == HWSERVO_OK) {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 MOTOR_NAME ": moving to %.1f deg over %u ms\r\n", deg, t_ms);
    } else {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 MOTOR_NAME ": move failed (err %d)\r\n", (int)st);
    }
    return pdFALSE;
}

static BaseType_t prvReadPosCommand(char *pcWriteBuffer, size_t xWriteBufferLen,
                                    const char *pcCommandString)
{
    (void)pcCommandString;

    float deg    = 0.0f;
    int16_t raw  = 0;

    hwservo_status_t st = HWSERVO_ReadAngle_deg(&servo, &deg);
    if (st == HWSERVO_OK) {
        HWSERVO_ReadPos_Raw(&servo, &raw);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 MOTOR_NAME ": %.1f deg (raw=%d)\r\n", deg, raw);
    } else {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 MOTOR_NAME ": read failed (err %d)\r\n", (int)st);
    }
    return pdFALSE;
}

static BaseType_t prvTorqueCommand(char *pcWriteBuffer, size_t xWriteBufferLen,
                                   const char *pcCommandString)
{
    BaseType_t xLen = 0;
    const char *pcArg = FreeRTOS_CLIGetParameter(pcCommandString, 1, &xLen);

    if (pcArg == NULL) {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Usage: torque <on|off>\r\n");
        return pdFALSE;
    }

    bool enable;
    if (strncmp(pcArg, "on", xLen) == 0) {
        enable = true;
    } else if (strncmp(pcArg, "off", xLen) == 0) {
        enable = false;
    } else {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "Invalid argument '%.*s'. Use 'on' or 'off'.\r\n", (int)xLen, pcArg);
        return pdFALSE;
    }

    hwservo_status_t st = HWSERVO_EnableTorque(&servo, enable);
    snprintf(pcWriteBuffer, xWriteBufferLen,
             MOTOR_NAME ": torque %s%s\r\n",
             enable ? "enabled" : "disabled",
             st == HWSERVO_OK ? "" : " (FAILED)");
    return pdFALSE;
}
