/**
  *****************************************************************************
  * @file    ledTask.c
  * @author  Jacky Lim
  * @brief   LED Task implementation. Mainly used for debugging and testing purposes.
  ******************************************************************************
  */

#include "debug.h"
#include "logger.h"

void ledTask(void const * argument)
{
    /* USER CODE BEGIN ledTask */
    for(;;)
    {
        HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_PIN);
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 sec
    }
    /* USER CODE END ledTask */
}