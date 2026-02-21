#ifndef __BSP_H
#define __BSP_H

#include "main.h"
#include "stdbool.h"

/* PIN Definition */
#define LED_PIN GPIO_PIN_3
#define LED_GPIO_PORT GPIOB

/* UART Debug Channel */
#define DEBUG_UART &huart2

#endif /* __BSP_H */
