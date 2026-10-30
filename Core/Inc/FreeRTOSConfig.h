#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include "stm32f4xx.h"

/* ================= 1. 基础配置 ================= */
#define configUSE_PREEMPTION                     1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION  1
#define configUSE_TICKLESS_IDLE                  0
#define configCPU_CLOCK_HZ                       ( 168000000UL ) /* F407 主频 168MHz */
#define configTICK_RATE_HZ                       ( 1000 )        /* 系统节拍 1kHz (1ms) */
#define configMAX_PRIORITIES                     ( 7 )           /* 最大任务优先级 (0~6) */
#define configMINIMAL_STACK_SIZE                 ( ( unsigned short ) 128 )
#define configMAX_TASK_NAME_LEN                  ( 16 )
#define configUSE_16_BIT_TICKS                   0
#define configIDLE_SHOULD_YIELD                  1
#define configUSE_TASK_NOTIFICATIONS             1
#define configUSE_MUTEXES                        1
#define configUSE_RECURSIVE_MUTEXES              1
#define configUSE_COUNTING_SEMAPHORES            1
#define configQUEUE_REGISTRY_SIZE                8

/* ================= 2. 内存管理 (匹配 heap_4.c) ================= */
#define configSUPPORT_STATIC_ALLOCATION          0
#define configSUPPORT_DYNAMIC_ALLOCATION         1
#define configTOTAL_HEAP_SIZE                    ( ( size_t ) ( 30 * 1024 ) ) /* 分配 30KB 给 RTOS 堆 */
#define configAPPLICATION_ALLOCATED_HEAP         0

/* ================= 3. 硬件 FPU 支持 (FOC 必须) ================= */
/* 确保启用了硬件浮点单元，避免进入 HardFault */
#define configENABLE_FPU                         1

/* ================= 4. 中断优先级配置 (极其重要！FOC 生死线) ================= */
/* STM32F4 默认使用 4 位抢占优先级 */
#ifdef __NVIC_PRIO_BITS
    #define configPRIO_BITS                      __NVIC_PRIO_BITS
#else
    #define configPRIO_BITS                      4
#endif

/* 最低优先级 (15) 给 SysTick 和 PendSV */
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY      15
#define configKERNEL_INTERRUPT_PRIORITY             0xF0

/* 
 * 【核心红线】：FreeRTOS 能管理的最高中断优先级。
 * 设为 5，意味着 NVIC 优先级数值为 0, 1, 2, 3, 4 的中断，绝对不能调用 FromISR API！
 * 你的 ADC (FOC) 优先级是 1，所以 ADC 中断里绝不能调用 xQueueSendFromISR 等，会死机。
 * 你的 EXTI 急停优先级是 5，刚好可以调用 FromISR API。
 */
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5
#define configMAX_SYSCALL_INTERRUPT_PRIORITY        0x50

/* ================= 5. 钩子函数与调试 ================= */
#define configUSE_IDLE_HOOK                      0
#define configUSE_TICK_HOOK                      0
#define configCHECK_FOR_STACK_OVERFLOW           0 /* 栈溢出检查 (开发阶段强烈建议设为2) */
#define configUSE_MALLOC_FAILED_HOOK             0 /* 内存分配失败钩子 */
#define configUSE_DAEMON_TASK_STARTUP_HOOK       0

/* ================= 6. 运行时统计与追踪 ================= */
#define configGENERATE_RUN_TIME_STATS            0
#define configUSE_TRACE_FACILITY                 1
#define configUSE_STATS_FORMATTING_FUNCTIONS     1

/* ================= 7. 协程 (不用，节省空间) ================= */
#define configUSE_CO_ROUTINES                    0
#define configMAX_CO_ROUTINE_PRIORITIES          ( 2 )

/* ================= 8. 软件定时器 ================= */
#define configUSE_TIMERS                         0
#define configTIMER_TASK_PRIORITY                ( configMAX_PRIORITIES - 1 ) /* 定时器任务优先级最高 */
#define configTIMER_QUEUE_LENGTH                 10
#define configTIMER_TASK_STACK_DEPTH             ( configMINIMAL_STACK_SIZE * 2 )

/* ================= 9. 可选 API 裁剪 (1=启用, 0=禁用) ================= */
#define INCLUDE_vTaskPrioritySet                 1
#define INCLUDE_uxTaskPriorityGet                1
#define INCLUDE_vTaskDelete                      1
#define INCLUDE_vTaskSuspend                     1
#define INCLUDE_xTaskDelayUntil                  1
#define INCLUDE_vTaskDelay                       1
#define INCLUDE_xTaskGetSchedulerState           1
#define INCLUDE_xTaskGetCurrentTaskHandle        1
#define INCLUDE_uxTaskGetStackHighWaterMark      1

/* ================= 10. 中断服务函数映射 (极其关键) ================= */
/* 告诉 FreeRTOS，这三个中断由它自己接管。因此你必须在 stm32f4xx_it.c 中把它们注释掉 */
#define vPortSVCHandler                          SVC_Handler
#define xPortPendSVHandler                       PendSV_Handler
#define xPortSysTickHandler                      SysTick_Handler

#endif /* FREERTOS_CONFIG_H */