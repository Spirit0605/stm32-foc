/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    can.h
 * @brief   This file contains all the function prototypes for
 *          the can.c file
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
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN Private defines */
  /*
    CAN ID	数据	DLC	缩放比例
    0x001	电机状态、CAN状态	2	原始值
    0x201	目标转速、实际转速、Iq_ref、Iq	8	RPM直接传，电流×1000
    0x202	Ia、Ib、Ic	6	×1000
    0x203	Vbus、Ibus、NTC	6	建议分别×100、×1000、×100
    0x204	三相PWM占空比	6	×100
    0x205	vd、vq、valpha、vbeta	8	×1000
  */

  typedef struct
  {
    uint16_t id;
    uint8_t dlc;
    uint8_t data[8];
  } CAN_Frame_t;

/* USER CODE END Private defines */

void MX_CAN1_Init(void);

/* USER CODE BEGIN Prototypes */
  void CAN_Write_16(uint8_t *data, int16_t value);
  HAL_StatusTypeDef CAN_SendFrame(CAN_Frame_t *frame);
  HAL_StatusTypeDef CAN_SendFrameWait(CAN_Frame_t *frame);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

