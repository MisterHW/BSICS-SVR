/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f7xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f7xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
/** Configurable Fault Status Register - CFSR */
typedef struct __attribute__ ((__packed__))
{
    /** MemManage Fault Status Sub-register - MMFSR */
    uint32_t iaccViol    : 1;  /**< MemManage Fault - The instruction access violation flag */
    uint32_t daccViol    : 1;  /**< MemManage Fault - The data access violation flag */
    uint32_t reserved1   : 1;  /**< Reserved */
    uint32_t mUnstkErr   : 1;  /**< MemManage Fault - Unstacking for a return from exception */
    uint32_t mStkErr     : 1;  /**< MemManage Fault - MemManage fault on stacking for exception entry */
    uint32_t mlspErr     : 1;  /**< MemManage Fault - MemManage fault occurred during floating-point lazy state preservation */
    uint32_t reserved2   : 1;  /**< Reserved */
    uint32_t mmarValid   : 1;  /**< MemManage Fault - The MemManage Address register valid flag */
    /** Bus Fault Status Sub-register - UFSR */
    uint32_t iBusErr     : 1;  /**< Bus Fault - The instruction bus error */
    uint32_t precisErr   : 1;  /**< Bus Fault - The precise Data bus error */
    uint32_t imprecisErr : 1;  /**< Bus Fault - The imprecise data bus error */
    uint32_t unstkErr    : 1;  /**< Bus Fault - Unstacking for an exception return has caused one or more bus faults */
    uint32_t stkErr      : 1;  /**< Bus Fault - Stacking for an exception entry has caused one or more bus faults */
    uint32_t lspErr      : 1;  /**< Bus Fault - A bus fault occurred during the floating-point lazy state */
    uint32_t reserved3   : 1;  /**< Reserved */
    uint32_t bfarValid   : 1;  /**< Bus Fault - The bus fault address register valid flag */
    /** Usage Fault Status Sub-register - UFSR */
    uint32_t undefInstr  : 1;  /**< Usage Fault - An undefined instruction */
    uint32_t invState    : 1;  /**< Usage Fault - The invalid state */
    uint32_t invPC       : 1;  /**< Usage Fault - An invalid PC */
    uint32_t noCP        : 1;  /**< Usage Fault - No coprocessor */
    uint32_t reserved4   : 4;  /**< Reserved */
    uint32_t unaligned   : 1;  /**< Usage Fault - Unaligned access */
    uint32_t divByZero   : 1;  /**< Usage Fault - Divide by zero */
    uint32_t reserved5   : 6;  /**< Reserved */
} cy_stc_fault_cfsr_t;
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ETH_HandleTypeDef heth;
extern TIM_HandleTypeDef htim1;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M7 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
    printf("\r\n\r\nHardFault_Handler called.\r\n");
    /* See PM0253 and SCB registers https://developer.arm.com/documentation/dui0646/a/CIHFDJCA .
     * HFSR 0xE000ED2C : HardFault Status Register (dword access)
     */
  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */
    printf("\r\n\r\nMemManage_Handler called.\r\n");
    /* See PM0253 and SCB registers https://developer.arm.com/documentation/dui0646/a/CIHFDJCA .
     * MMFSR 0xE000ED28 : MemManage Fault Status register (byte access)
     * MMFAR 0xE000ED34 : MemManage Fault Address Register (dword access)
     * cy_stc_fault_cfsr_t sr = SCB->CFSR;
     */
    printf("MMFSR : 0x%2x\r\n", SCB->CFSR & 0xFF);
    printf("MMFAR : 0x%8x\r\n", SCB->MMFAR);

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */
    printf("\r\n\r\nBusFault_Handler called.\r\n");
    /* See PM0253 and SCB registers https://developer.arm.com/documentation/dui0646/a/CIHFDJCA .
     * BFSR 0xE000ED29 : Bus Fault Status register (byte access)
     * BFAR 0xE000ED38 : BusFault Address Register
     */
    printf("BFSR : 0x%2x\r\n", (SCB->CFSR >> 8) & 0xFF);
    printf("BFAR : 0x%8x\r\n", SCB->BFAR);
  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */
    printf("\r\n\r\nUsageFault_Handler called.\r\n");
    /* See PM0253 and SCB registers https://developer.arm.com/documentation/dui0646/a/CIHFDJCA .
     * UFSR 0xE000ED2A : UsageFault Status Register (word access)
     */
    printf("UFSR : 0x%4x\r\n", (SCB->CFSR >> 16) & 0xFFFF);
  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32F7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f7xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
  */
void TIM1_UP_TIM10_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}

/**
  * @brief This function handles Ethernet global interrupt.
  */
void ETH_IRQHandler(void)
{
  /* USER CODE BEGIN ETH_IRQn 0 */

  /* USER CODE END ETH_IRQn 0 */
  HAL_ETH_IRQHandler(&heth);
  /* USER CODE BEGIN ETH_IRQn 1 */

  /* USER CODE END ETH_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
