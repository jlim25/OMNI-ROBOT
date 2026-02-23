#pragma once

/**
  *****************************************************************************
  * @file    cli_commands.h
  * @brief   Declarations for application CLI command registration.
  *
  *          Call CLI_RegisterAllCommands() once at startup (before the
  *          CLI console task begins processing input) to register every
  *          application command with the FreeRTOS-Plus-CLI interpreter.
  *****************************************************************************
  */

/**
 * Register all application-defined CLI commands with FreeRTOS-Plus-CLI.
 * Call once during initialisation, before starting the CLI console task.
 */
void CLI_RegisterAllCommands(void);
