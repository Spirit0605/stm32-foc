/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "math.h"
#include "AS5600.h"
#include "current.h"
#include "foc.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// 任务句柄

TaskHandle_t AS5600Handle = NULL; // 编码器采取任务
TaskHandle_t ADCHandle = NULL;    // ADC采样任务

volatile uint8_t stop_flag = 0;   // 急停标志位：0正常，1急停
volatile uint8_t start_flag = 0;  // 已经上电标志位
volatile uint8_t off_finshal = 0; // 零点偏置标志位

volatile uint16_t adc_Regular_buffer[3]; // 存储ADC规则组数据 V-BUS 母线电压 C-BUS 母线电流 NTC 温度采样
// volatile uint16_t adc_Inject_buffer[3];  // 存储ADC注入组数据 Ia, Ib, Ic
// volatile uint16_t adc_cnt = 0;           // adc注入组转换次数
volatile float v_bus = 0.0f; // ⺟线电压采样输出，对应 ADC_VBUS
volatile float c_bus = 0.0f; // ⺟线电流采样输出，对应 ADC_CBUS
volatile float ntc = 0.0f;   // 温度采样输出，对应 ADC_NTC

// 零点偏置校准
volatile uint16_t of_a = 0, of_b = 0, of_c = 0;
// 唤醒驱动板后
// 第1次： of_a = 2095, of_b = 2089, of_c = 2091
// 第2次： of_a = 2096, of_b = 2088, of_c = 2092
// 第3次： of_a = 2098, of_b = 2090, of_c = 2092
// 第4次： of_a = 2097, of_b = 2088, of_c = 2090
// 第5次： of_a = 2095, of_b = 2087, of_c = 2091
// 第6次： of_a = 2097, of_b = 2087, of_c = 2090
// 第7次： of_a = 2096, of_b = 2089, of_c = 2091
// 第8次： of_a = 2098, of_b = 2089, of_c = 2090
// 第9次： of_a = 2099, of_b = 2089, of_c = 2090
// 第10次： of_a = 2097, of_b = 2087, of_c = 2092

int16_t raw_a = 0, raw_b = 0, raw_c = 0;
// 采样电流
volatile float Ia = 0.0f, Ib = 0.0f, Ic = 0.0f; // 三相采样电流
volatile float elec_rad = 0.0f;                 // 电角度（弧度制）
//
float ialpha = 0.0f;
float ibeta = 0.0f;

float id = 0.0f;
float iq = 0.0f;

static PI_Controller_t pi_id;
static PI_Controller_t pi_iq;
static FOC_Control_t foc_ctrl;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* 根据你的实际 PWM/电源参数设置 */
#define FOC_DT 0.00005f // 示例：20 kHz 控制周期
#define FOC_VBUS 12.0f  // 示例值，改成实测母线电压
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void Led_Toggle()
{
  HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
  HAL_Delay(100);
}

static void FOC_Control_Init(void)
{
  /* 初期先用较小的输出限幅，后续结合硬件参数调整 */
  PI_Init(&pi_id, 0.3f, 50.0f, -2.0f, 2.0f);
  PI_Init(&pi_iq, 0.3f, 50.0f, -2.0f, 2.0f);

  foc_ctrl.id_ref = 0.0f;
  foc_ctrl.iq_ref = 0.0f;
}

// 编码器采集任务
void vAS5600Task(void *argument)
{
  (void)argument;
  AS5600_DataTypeDef enc;

  // 初始化，失败就重试
  while (as5600_init() == 0)
  {
    printf("编码器初始化失败，重试中...\r\n");
    vTaskDelay(pdMS_TO_TICKS(200));
  }

  for (;;)
  {
    if (stop_flag == 1)
    {
      Led_Toggle();
      vTaskSuspend(NULL);
    }

    // I2C 失败就跳过
    if (as5600_update() == 1)
    {
      // 获得编码器读取数据
      as5600_get_data(&enc);
      elec_rad = enc.elec_angle; // 电角度
    }
    vTaskDelay(pdMS_TO_TICKS(2));
    // 机械角度、电角度转成角度制显示，方便观察
    // float mech_deg = enc.mech_angle * 180.0f / PI;
    // float elec_deg = enc.elec_angle * 180.0f / PI;
    // float turn = (float)enc.total / AS5600_RESOLUTION;
    // int32_t rpm_x10 = (int32_t)(enc.rpm * 10.0f);
    // printf("raw=%4u  mech=%6.1f°  elec=%6.1f°  turn=%+.2f  rpm=%+ld.%01ld\r\n",
    //        enc.raw, mech_deg, elec_deg, turn,rpm_x10 / 10, (rpm_x10 < 0 ? -rpm_x10 : rpm_x10) % 10);
  }
}

// ADC采集任务
void vADCTask(void *argument)
{
  (void)argument;

  uint32_t sum_a = 0, sum_b = 0, sum_c = 0;
  uint16_t cnt = 0;

  while (cnt < 2000)
  {
    sum_a += raw_a;
    sum_b += raw_b;
    sum_c += raw_c;
    cnt++;

    vTaskDelay(pdMS_TO_TICKS(1));
  }

  of_a = sum_a / cnt;
  of_b = sum_b / cnt;
  of_c = sum_c / cnt;
  off_finshal = 1;

  printf("of_a = %d, of_b = %d, of_c = %d\n", of_a, of_b, of_c);

  for (;;)
  {
    if (stop_flag == 1)
    {
      Led_Toggle();
      vTaskSuspend(NULL);
    }

    // printf("VBus: %.3f, CBus: %.3f, NTC: %.3f\n", adc_Regular_buffer[0], adc_Regular_buffer[2], adc_Regular_buffer[2]);
    printf("VBus: %.3f, CBus: %.3f, NTC: %.3f\n", v_bus, c_bus, ntc);

    printf("ialpha: %.3f, ibeta: %.3f, id: %.3f, iq: %.3f\n", ialpha, ibeta, id, iq);

    // 挂起任务
    //  vTaskSuspend(NULL);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// 创建任务
void vOtherFunction(void)
{
  BaseType_t xAS5600Returned = xTaskCreate(vAS5600Task, "AS5600", 512, NULL, 1, &AS5600Handle);
  BaseType_t xADCReturned = xTaskCreate(vADCTask, "ADC", 512, NULL, 1, &ADCHandle);

  // if (xReturned != pdPASS)
  // {
  // }
}

/* HAL Tick 使用 TIM6，周期 1ms */
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
  __HAL_RCC_TIM6_CLK_ENABLE();

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 84 - 1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1000 - 1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    return HAL_ERROR;
  }

  HAL_NVIC_SetPriority(TIM6_DAC_IRQn, TickPriority, 0);
  HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);

  return HAL_TIM_Base_Start_IT(&htim6);
}

void HAL_SuspendTick(void)
{
  __HAL_TIM_DISABLE_IT(&htim6, TIM_IT_UPDATE);
}

void HAL_ResumeTick(void)
{
  __HAL_TIM_ENABLE_IT(&htim6, TIM_IT_UPDATE);
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  // 1.确保 DRV_EN 和 CAL 为低
  HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(CAL_GPIO_Port, CAL_Pin, GPIO_PIN_RESET);

  // 2.可以通电，led灯亮
  HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

  // 3.等待按键按下，确认通电，led灯灭
  while (1)
  {
    HAL_Delay(200);
    if (start_flag)
    {
      HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
      start_flag = 0;
      break;
    }
  }

  // 4.拉高DRV_EN
    HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, GPIO_PIN_SET);

  // 5.唤醒完成后确认 FAULT 为⾼电平
  while (HAL_GPIO_ReadPin(FAULT_GPIO_Port, FAULT_Pin) == 0)
  {
    HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
    HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    HAL_Delay(100);
  }

  // 6.启动慢速规则组 DMA 采样
  uint8_t rct =  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_Regular_buffer, 3);
  // printf("ADC DMA start status=%d\r\n", (int)rct);

  // 7.启动注入组，等待 TIM1 硬件触发
  HAL_ADCEx_InjectedStart_IT(&hadc1);

  // 8.启动 TIM1 的 6 路 PWM，同时函数自动启动tim1
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

  // 9.必须手动开启主输出 MOE
  __HAL_TIM_MOE_ENABLE(&htim1);

  HAL_Delay(1000); // 等待采样电路稳定
  FOC_Control_Init();

  // 10.任务创建
  vOtherFunction();

  // 11.启动FreeRTOS调度器
  vTaskStartScheduler();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

// 按键外部中断 - 急停
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == K3_Pin)
  {
    // 1.防抖 - 停止接收中断
    HAL_NVIC_DisableIRQ(EXTI4_IRQn);

    // 2.开启急停标志
    stop_flag = 1;

    // 3.相关硬件设备关闭

    // 拉低DRV_EN，DRV8323会瞬间进入休眠，封锁所有MOE管输出
    HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, GPIO_PIN_RESET);
    // 关闭 TIM1 的 PWM 主输出水闸 (MOE)
    __HAL_TIM_MOE_DISABLE(&htim1);
    // 停止ADC采样转换
    HAL_ADC_Stop_DMA(&hadc1);
    HAL_ADCEx_InjectedStop_IT(&hadc1);
    // 停止PWM通道 (双保险)
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);

    // 4.点亮故障指示灯
  }
  else if (GPIO_Pin == K1_Pin)
  {
    start_flag = 1;
  }
}
// ADC 注入通道转换完成回调函数
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) {
        v_bus = FOC_GetVbus(adc_Regular_buffer[0]);
        c_bus = FOC_GetCbus(adc_Regular_buffer[1]);
        ntc   = FOC_GetNTC(adc_Regular_buffer[2]);
    }
}

// ADC 注入通道转换完成回调函数
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    // 读取ADC注入组的采集的ABC三相电流采样输出
    raw_a = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
    raw_b = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);
    raw_c = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_3);

    Ia = CurrentSense_Convert(raw_a, of_a);
    Ib = CurrentSense_Convert(raw_b, of_b);
    Ic = CurrentSense_Convert(raw_c, of_c);

    if (off_finshal != 0)
    {
      FOC_Clarke(Ia, Ib, &ialpha, &ibeta);
      FOC_Park(ialpha, ibeta, elec_rad, &id, &iq);

      // 电流双闭环
      foc_ctrl.vd = PI_Update(&pi_id, foc_ctrl.id_ref, id, FOC_DT);

      foc_ctrl.vq = PI_Update(&pi_iq, foc_ctrl.iq_ref, iq, FOC_DT);

      // 反Park
      FOC_InversePark(foc_ctrl.vd, foc_ctrl.vq, elec_rad, &foc_ctrl.valpha, &foc_ctrl.vbeta);

      // SVPWM：先计算占空比，不写 TIM1 CCR
      FOC_SVPWM(foc_ctrl.valpha, foc_ctrl.vbeta, v_bus, &foc_ctrl.duty_a, &foc_ctrl.duty_b, &foc_ctrl.duty_c);

      //更新占空比
      TIM1->CCR1 = (uint32_t)(foc_ctrl.duty_a * (__HAL_TIM_GET_AUTORELOAD(&htim1) + 1U));
      TIM1->CCR2 = (uint32_t)(foc_ctrl.duty_b * (__HAL_TIM_GET_AUTORELOAD(&htim1) + 1U));
      TIM1->CCR3 = (uint32_t)(foc_ctrl.duty_c * (__HAL_TIM_GET_AUTORELOAD(&htim1) + 1U));
    }
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
