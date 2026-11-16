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
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "arm_math.h"
#include "AS5600.h"
#include "adc_signal.h"
#include "foc.h"
#include "delay.h"

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

TaskHandle_t AS5600Handle = NULL;    // 编码器采取任务
TaskHandle_t LogHandle = NULL;       // 串口打印任务
TaskHandle_t SpeedLoopHandle = NULL; // 速度环任务句柄
TaskHandle_t CanHandle = NULL;       // CAN任务句柄

static float open_loop_theta = 0.0f;

volatile Motors_State motors_state = MotorsStop; // 电机状态，默认停止
volatile Can_States can_state = CanStop;         // Can总线状态，默认停止

volatile uint16_t adc_Regular_buffer[3]; // 存储ADC规则组数据 V-BUS 母线电压 C-BUS 母线电流 NTC 温度采样

volatile float v_bus = 0.0f; // ⺟线电压采样输出，对应 ADC_VBUS
volatile float c_bus = 0.0f; // ⺟线电流采样输出，对应 ADC_CBUS
volatile float ntc = 0.0f;   // 温度采样输出，对应 ADC_NTC

// 电角度零点偏置校准
volatile float electrical_offset = 0.0f;
volatile float real_elec_rad = 0.0f;
// 零点偏置校准
volatile uint16_t of_a = 0, of_b = 0, of_c = 0;

int16_t raw_a = 0, raw_b = 0, raw_c = 0;
// 采样电流
volatile float Ia = 0.0f, Ib = 0.0f, Ic = 0.0f; // 三相采样电流
volatile float elec_rad = 0.0f;                 // 电角度（弧度制）
volatile float now_rpm = 0.0f;                  // 电角度转速（RPM）

volatile float ialpha = 0.0f;
volatile float ibeta = 0.0f;

float id = 0.0f;
float iq = 0.0f;

volatile AS5600_DataTypeDef enc;
PI_Controller_t pi_id;    // 电流环
PI_Controller_t pi_iq;    // 电流环
PI_Controller_t pi_speed; // 速度环 PI 控制器结构体实例
FOC_Control_t foc_ctrl;

volatile float32_t target_rpm = 0.0f; // 目标转速
volatile float32_t Iq_ref = 0.0f;     // 目标电流 Iq_ref，来自速度环输出，作为电流环的目标值

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* 根据你的实际 PWM/电源参数设置 */
#define FOC_DT 0.00005f      // 示例：20 kHz 控制周期
#define SPEED_LPF_ALPHA 0.2f // 速度一阶低通滤波系数（取值 0~1，越小抗噪越强但滞后越大）

#define Motor_Speed 100.0f // 目标转速

#define KP_INIT 0.5f
#define KI_INIT 0.05f
#define OUT_MIN -10.0f
#define OUT_MAX 10.0f

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
  delay_ms(100);
  HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
  delay_ms(100);
  HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
  delay_ms(100);
}

void Get_Current_Offset(void)
{
  uint32_t sum_a = 0, sum_b = 0, sum_c = 0;
  uint16_t cnt = 0;

  float align_valpha = 1.5f;
  float align_vbeta = 0.0f;
  float d_a, d_b, d_c;

  // AB相采样电流偏置
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_Regular_buffer, 3);
  HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_4);
  __HAL_TIM_MOE_DISABLE(&htim1);

  while (cnt < 2000)
  {
    // 软件手动单次触发注入组采样
    HAL_ADCEx_InjectedStart(&hadc1);

    // 等待 ADC 硬件采样转换完成（超时设为 10ms）
    if (HAL_ADCEx_InjectedPollForConversion(&hadc1, 10) == HAL_OK)
    {
      sum_a += HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
      sum_b += HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
      sum_c += HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3);
      cnt++;
    }
    delay_us(100);
  }

  of_a = sum_a / cnt;
  of_b = sum_b / cnt;
  of_c = sum_c / cnt;

  // 获取真实母线电压（单位：V），防止传原始ADC码值
  float real_vbus = FOC_GetVbus(adc_Regular_buffer[0]);
  if (real_vbus < 1.0f)
  {
    real_vbus = 12.0f; // 兜底，防止除零
  }

  // 电角度0°偏置校准
  FOC_SVPWM(align_valpha, align_vbeta, real_vbus, &d_a, &d_b, &d_c);
  TIM1->CCR1 = (uint32_t)(d_a * (__HAL_TIM_GET_AUTORELOAD(&htim1) + 1U));
  TIM1->CCR2 = (uint32_t)(d_b * (__HAL_TIM_GET_AUTORELOAD(&htim1) + 1U));
  TIM1->CCR3 = (uint32_t)(d_c * (__HAL_TIM_GET_AUTORELOAD(&htim1) + 1U));

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

  __HAL_TIM_MOE_ENABLE(&htim1);
  HAL_TIM_Base_Start(&htim1);

  delay_ms(1000);

  if (as5600_update() == 1)
  {
    // 获得编码器读取数据
    as5600_get_data(&enc);
    electrical_offset = enc.elec_angle; // 电角度
  }

  printf("Offset_A = %d, Offset_B = %d, Offset_C = %d, Offset_Elec = %.3f\n", of_a, of_b, of_c, electrical_offset);
}

// 编码器采集任务
void vAS5600Task(void *argument)
{
  (void)argument;

  for (;;)
  {
    if (motors_state == MotorsStop)
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
      now_rpm = enc.rpm;         // 电角度转速
    }

    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_Regular_buffer, 3);

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// ADC采集任务
void vLogTask(void *argument)
{
  (void)argument;

  for (;;)
  {
    if (motors_state == MotorsStop)
    {
      Led_Toggle();
      vTaskSuspend(NULL);
    }
    // 测试计算耗时
    // printf("delay = %d\n", delay);B

    printf("Raw: A = %d, B = %d, C = %d, Ia: %.3f, Ib: %.3f, Ic: %.3f\n", raw_a, raw_b, raw_c, Ia, Ib, Ic);

    // printf("电角度: %.3f, 转速: %.3f\n", real_elec_rad, now_rpm);

    // printf("Ia: %.3f, Ib: %.3f, Ic: %.3f\n", Ia, Ib, Ic);

    printf("VBus: %.3f, CBus: %.3f, NTC: %.3f\n", v_bus, c_bus, ntc);

    // printf("ialpha: %.3f, ibeta: %.3f, id: %.3f, iq: %.3f\n", ialpha, ibeta, id, iq);

    // printf("vd: %.3f, vq: %.3f, duty_a: %.3f, duty_b: %.3f, duty_c: %.3f\n", foc_ctrl.vd, foc_ctrl.vq, foc_ctrl.duty_a, foc_ctrl.duty_b, foc_ctrl.duty_c);

    // printf("目标转速: %.3f, 目标电流Iq_ref: %.3f\n", target_rpm, Iq_ref);

    // printf("pwm: A = %.3f, B = %.3f, c = %.3f\n", foc_ctrl.duty_a, foc_ctrl.duty_b, foc_ctrl.duty_c);

    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

// 速度环
void vSpeedLoopTask(void *argument)
{
  (void)argument;
  float32_t filtered_rpm = 0.0f;

  for (;;)
  {
    if (motors_state == MotorsStop)
    {
      Led_Toggle();
      vTaskSuspend(NULL);
    }

    if (motors_state == MotorsStart)
    {
      float32_t raw_rpm = now_rpm;

      // 一阶低通滤波
      filtered_rpm += SPEED_LPF_ALPHA * (raw_rpm - filtered_rpm);

      // dt = 0.01s（10ms）
      float32_t target_iq = PI_Update(&pi_speed, target_rpm, filtered_rpm, 0.01f);
      Iq_ref = target_iq;
    }
    else
    {
      // 非运行状态强制清零，防止积分冲击
      pi_speed.integral = 0.0f;
      Iq_ref = 0.0f;
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void vCANTask(void *argument)
{
  (void)argument;

  CAN_Frame_t frame = {0};
  HAL_StatusTypeDef can_status = HAL_OK;

  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    can_state = CanError;
    printf("CAN States is Failed!\n");
    vTaskDelete(NULL);
  }
  can_state = CanRunning;

  for (;;)
  {
    if (motors_state == MotorsStop)
    {
      Led_Toggle();
      vTaskSuspend(NULL);
    }

    can_status = HAL_OK;

    frame.id = 0x001;
    frame.dlc = 2;
    frame.data[0] = motors_state;
    frame.data[1] = can_state;
    if (CAN_SendFrameWait(&frame) != HAL_OK)
    {
      can_status = HAL_ERROR;
    }

    // 转速/电流
    frame.id = 0x201;
    frame.dlc = 8;
    CAN_Write_16(&frame.data[0], (int16_t)target_rpm);
    CAN_Write_16(&frame.data[2], (int16_t)now_rpm);
    CAN_Write_16(&frame.data[4], (int16_t)(Iq_ref * 1000));
    CAN_Write_16(&frame.data[6], (int16_t)(iq * 1000));
    if (CAN_SendFrameWait(&frame) != HAL_OK)
    {
      can_status = HAL_ERROR;
    }

    // 三相采样电流
    frame.id = 0x202;
    frame.dlc = 6;
    CAN_Write_16(&frame.data[0], (int16_t)(Ia * 1000));
    CAN_Write_16(&frame.data[2], (int16_t)(Ib * 1000));
    CAN_Write_16(&frame.data[4], (int16_t)(Ic * 1000));
    if (CAN_SendFrameWait(&frame) != HAL_OK)
    {
      can_status = HAL_ERROR;
    }

    // 母线电压/电流/温度
    frame.id = 0x203;
    frame.dlc = 6;
    CAN_Write_16(&frame.data[0], (int16_t)(v_bus * 10));
    CAN_Write_16(&frame.data[2], (int16_t)(c_bus * 1000));
    CAN_Write_16(&frame.data[4], (int16_t)(ntc * 100));
    if (CAN_SendFrameWait(&frame) != HAL_OK)
    {
      can_status = HAL_ERROR;
    }

    // PWM占空比
    frame.id = 0x204;
    frame.dlc = 6;
    CAN_Write_16(&frame.data[0], (int16_t)(foc_ctrl.duty_a * 100));
    CAN_Write_16(&frame.data[2], (int16_t)(foc_ctrl.duty_b * 100));
    CAN_Write_16(&frame.data[4], (int16_t)(foc_ctrl.duty_c * 100));
    if (CAN_SendFrameWait(&frame) != HAL_OK)
    {
      can_status = HAL_ERROR;
    }

    // vd/vq/valpha/vbeta
    frame.id = 0x205;
    frame.dlc = 8;
    CAN_Write_16(&frame.data[0], (int16_t)(foc_ctrl.vd * 1000));
    CAN_Write_16(&frame.data[2], (int16_t)(foc_ctrl.vq * 1000));
    CAN_Write_16(&frame.data[4], (int16_t)(foc_ctrl.valpha * 1000));
    CAN_Write_16(&frame.data[6], (int16_t)(foc_ctrl.vbeta * 1000));
    if (CAN_SendFrameWait(&frame) != HAL_OK)
    {
      can_status = HAL_ERROR;
    }

    can_state = (can_status == HAL_OK) ? CanRunning : CanError;

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// 创建任务
void vOtherFunction(void)
{
  BaseType_t xAS5600Returned = xTaskCreate(vAS5600Task, "AS5600", 512, NULL, 4, &AS5600Handle);
  BaseType_t xLogReturned = xTaskCreate(vLogTask, "Log", 512, NULL, 0, &LogHandle);
  BaseType_t xSpeedLoopHandle = xTaskCreate(vSpeedLoopTask, "SpeedLoop", 512, NULL, 3, &SpeedLoopHandle);
  BaseType_t xCANTaskHandle = xTaskCreate(vCANTask, "CAN", 512, NULL, 2, &CanHandle);

  // if (xReturned != pdPASS)
  // {
  // }
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
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */

  /* 开启DWT周期计数器，只需执行一次 */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  // 初始化定时器tim6
  HAL_TIM_Base_Start(&htim6);

  // 初始化编码器
  while (as5600_init() == 0)
  {
    printf("编码器初始化失败，重试中...\r\n");
    delay_ms(200);
  }

  // 电流环PI初始化控制器
  PI_Init(&pi_id, KP_INIT, KI_INIT, OUT_MIN, OUT_MAX);
  PI_Init(&pi_iq, KP_INIT, KI_INIT, OUT_MIN, OUT_MAX);
  // 速度环PI初始化控制器
  // 参数 2 (Kp): 比例系数。控制响应速度。由于低通滤波有滞后，建议先给小值（如 0.01f 到 0.03f 盲调）
  // 参数 3 (Ki): 积分系数。消除静差。建议从 0.05f ~ 0.1f 开始调校
  PI_Init(&pi_speed, 0.005f, 0.05f, -1.0f, 1.0f);
  // 开机默认目标转速设为 0 RPM（静止）
  target_rpm = Motor_Speed;

  // 确保 DRV_EN 和 CAL 为低
  HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(CAL_GPIO_Port, CAL_Pin, GPIO_PIN_RESET);

  // 可以通电，led灯亮
  HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

  // 等待按键按下，确认通电，led灯灭
  do
  {
    delay_ms(200);
  } while (motors_state != MotorsReady);

  HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);

  // 拉高DRV_EN
  HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, GPIO_PIN_SET);

  // 唤醒完成后确认 FAULT 为⾼电平
  while (HAL_GPIO_ReadPin(FAULT_GPIO_Port, FAULT_Pin) == 0)
  {
    HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
    HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    delay_ms(100);
  }

  delay_ms(1000); // 等待采样电路稳定

  // 开始进行电流偏置校准 + 电角度0点校准
  Get_Current_Offset();
  motors_state = MotorsOffset;

  HAL_ADCEx_InjectedStart_IT(&hadc1);

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
  if (GPIO_Pin == KEY0_Pin)
  {
    // 1.防抖 - 停止接收中断
    HAL_NVIC_DisableIRQ(EXTI4_IRQn);

    // 2.开启急停标志
    motors_state = MotorsStop;

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
  }
  else if (GPIO_Pin == KEYUP_Pin)
  {
    motors_state = MotorsReady;
  }
}
// ADC 注入通道转换完成回调函数
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    v_bus = FOC_GetVbus(adc_Regular_buffer[0]);
    c_bus = FOC_GetCbus(adc_Regular_buffer[1]);
    ntc = FOC_GetNTC(adc_Regular_buffer[2]);
  }
}

volatile uint32_t foc_cycles = 0;     // 本次耗费的CPU周期数
volatile uint32_t foc_cycles_max = 0; // 记录观察期间的最大值

// ADC 注入通道转换完成回调函数
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  uint32_t start_cycles = DWT->CYCCNT;

  if (hadc->Instance == ADC1)
  {
    if (motors_state < MotorsOffset)
      return;
    if (motors_state == MotorsOffset)
      motors_state = MotorsStart;

    // 计算真实电角度并归一化（供电流环使用）
    real_elec_rad = elec_rad - electrical_offset;
    if (real_elec_rad < 0.0f)
      real_elec_rad += TWO_PI;
    if (real_elec_rad >= TWO_PI)
      real_elec_rad -= TWO_PI;

    // 读取ADC注入组的采集的ABC三相电流采样输出  --这里只采AB相电流，换算出Ia和Ib后计算得出Ic
    raw_a = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
    raw_b = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);
    raw_c = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_3);

    Ia = CurrentSense_Convert(raw_a, of_a);
    Ib = CurrentSense_Convert(raw_b, of_b);
    Ic = CurrentSense_Convert(raw_c, of_c);

    FOC_Clarke(Ia, Ib, &ialpha, &ibeta);
    FOC_Park(ialpha, ibeta, real_elec_rad, &id, &iq);

    // 电流环 双闭环
    foc_ctrl.vd = PI_Update(&pi_id, 0.0f, id, FOC_DT);
    foc_ctrl.vq = PI_Update(&pi_iq, Iq_ref, iq, FOC_DT); // Iq_ref来自速度环

    // 反Park
    FOC_InversePark(foc_ctrl.vd, foc_ctrl.vq, real_elec_rad, &foc_ctrl.valpha, &foc_ctrl.vbeta);

    // SVPWM：先计算占空比，不写 TIM1 CCR
    FOC_SVPWM(foc_ctrl.valpha, foc_ctrl.vbeta, v_bus, &foc_ctrl.duty_a, &foc_ctrl.duty_b, &foc_ctrl.duty_c);

    // 更新占空比
    TIM1->CCR1 = (uint32_t)(foc_ctrl.duty_a * (__HAL_TIM_GET_AUTORELOAD(&htim1) + 1U));
    TIM1->CCR2 = (uint32_t)(foc_ctrl.duty_b * (__HAL_TIM_GET_AUTORELOAD(&htim1) + 1U));
    TIM1->CCR3 = (uint32_t)(foc_ctrl.duty_c * (__HAL_TIM_GET_AUTORELOAD(&htim1) + 1U));
  }
  foc_cycles = DWT->CYCCNT - start_cycles;

  if (foc_cycles > foc_cycles_max)
  {
    foc_cycles_max = foc_cycles;
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
