/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.h
  * @brief   This file contains all the function prototypes for
  *          the gpio.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GPIO_H__
#define __GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#define READ_HW_OUT_1 HAL_GPIO_ReadPin(HW_OUT_1_GPIO_Port,HW_OUT_1_Pin)
#define READ_HW_OUT_2 HAL_GPIO_ReadPin(HW_OUT_2_GPIO_Port,HW_OUT_2_Pin)
#define READ_HW_OUT_3 HAL_GPIO_ReadPin(HW_OUT_3_GPIO_Port,HW_OUT_3_Pin)
#define READ_HW_OUT_4 HAL_GPIO_ReadPin(HW_OUT_4_GPIO_Port,HW_OUT_4_Pin)
/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_GPIO_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ GPIO_H__ */

