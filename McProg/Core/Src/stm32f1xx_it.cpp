/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "frequency_generation.h"

// C-Name mangling so that the linker finds the ISR functions
#ifdef __cplusplus
 extern "C" {
#endif


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

 S1Pwm* s1pwmPtr;
 uint16_t dutyMain;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

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

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

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

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
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

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(B1_Pin);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/* USER CODE BEGIN 1 */

void setS1PwmPtr(struct S1Pwm* s1pwm)
{
	s1pwmPtr = s1pwm;
}

void setDuty(uint16_t duty)
{
	dutyMain = duty;
}

void DMA1_Channel3_IRQHandler(void)
{
	// Transfer error interrupt
	if (DMA1->ISR & DMA_ISR_TEIF3) {
		DMA1->IFCR |= DMA_IFCR_CTEIF3;			// Clear interrupt
	}

	// Half transfer complete interrupt
	static uint16_t dmaBufferPeriodsLast;
	static uint16_t periodsInMasterPLast;

	if (DMA1->ISR & DMA_ISR_HTIF3) {
		DMA1->IFCR |= DMA_IFCR_CHTIF3;			// Clear interrupt

		bool end = false;
		for (size_t i=0; i<s1pwmPtr->dmaBufferPeriods; ++i) {
			for (size_t j=0; j<s1pwmPtr->periodsInMasterP; ++j) {
				S1DmaFieldT duty = 0x0;
				if (j<6)
					duty = dutyMain;
				else if (j > (s1pwmPtr->periodsInMasterP / 2) && j < (s1pwmPtr->periodsInMasterP / 2 + 6))
					duty = dutyMain;
				size_t idx = i*s1pwmPtr->periodsInMasterP + j;
				if (idx >= s1pwmPtr->no/2) {
					dmaBufferPeriodsLast = i;
					periodsInMasterPLast = j;
					end = true;
					break;
				}
				else if (idx < s1pwmPtr->no)
					s1pwmPtr->mem0[idx] = duty;
				//else
				//	assert(false);
			}
			if (end)
				break;
		}
	}

	// Transfer complete interrupt
	if (DMA1->ISR & DMA_ISR_TCIF3) {
		DMA1->IFCR |= DMA_IFCR_CTCIF3;			// Clear interrupt

		for (size_t i=dmaBufferPeriodsLast; i<s1pwmPtr->dmaBufferPeriods; ++i) {
			for (size_t j=periodsInMasterPLast; j<s1pwmPtr->periodsInMasterP; ++j) {
				S1DmaFieldT duty = 0x0;
				if (j<6)
					duty = dutyMain;
				else if (j > (s1pwmPtr->periodsInMasterP / 2) && j < (s1pwmPtr->periodsInMasterP / 2 + 6))
					duty = dutyMain;
				size_t idx = i*s1pwmPtr->periodsInMasterP + j;
				if (idx < s1pwmPtr->no)
					s1pwmPtr->mem0[idx] = duty;
				//else
				//	assert(false);
			}
		}
	}
}

#ifdef __cplusplus
}
#endif

/* USER CODE END 1 */
