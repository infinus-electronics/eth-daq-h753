/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32h7xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
extern void xPortSysTickHandler();
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
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi2_rx;
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern DMA_HandleTypeDef hdma_tim1_up;
extern DMA_HandleTypeDef hdma_tim3_up;
extern DMA_HandleTypeDef hdma_tim4_ch1;
extern DMA_HandleTypeDef hdma_tim4_up;
/* USER CODE BEGIN EV */
extern TaskHandle_t vADCTCPTaskHandle;
extern TaskHandle_t vAuxADCTCPTaskHandle;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
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
  * @brief This function handles Pre-fetch fault, memory access fault.
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
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */
  if( xTaskGetSchedulerState() == taskSCHEDULER_RUNNING )
      {
    	xPortSysTickHandler();
      }
  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32H7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h7xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream0 global interrupt.
  */
void DMA1_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream0_IRQn 0 */
  BaseType_t xHigherPriorityTaskWoken;
  uint32_t ulCurrBuf = 0;
  //SPI1 Stream
  if (DMA1->LISR & DMA_FLAG_TCIF0_4){
      uint32_t status = DMA1_Stream0->CR;
        if ((status & DMA_SxCR_CT_Msk)!=0){ //current Transmittable Buffer
  	  ulCurrBuf = 0;
        } else {
  	  ulCurrBuf = 1;
        }

    }
  /* USER CODE END DMA1_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi1_rx);
  /* USER CODE BEGIN DMA1_Stream0_IRQn 1 */
  /* xHigherPriorityTaskWoken must be initialised to pdFALSE. If calling
     xTaskNotifyFromISR() unblocks the handling task, and the priority of
     the handling task is higher than the priority of the currently running task,
     then xHigherPriorityTaskWoken will automatically get set to pdTRUE. */
  xHigherPriorityTaskWoken = pdFALSE;
  /* Unblock the handling task so the task can perform any processing necessitated
     by the interrupt. xHandlingTask is the task's handle, which was obtained
     when the task was created. The handling task's 0th notification value
     is bitwise ORed with the interrupt status - ensuring bits that are already
     set are not overwritten. */
  xTaskNotifyFromISR( vADCTCPTaskHandle,
			     ulCurrBuf,
			     eSetBits,
			     &xHigherPriorityTaskWoken );

  /* Force a context switch if xHigherPriorityTaskWoken is now set to pdTRUE.
     The macro used to do this is dependent on the port and may be called
     portEND_SWITCHING_ISR. */
  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
  /* USER CODE END DMA1_Stream0_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream1 global interrupt.
  */
void DMA1_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */
  BaseType_t xHigherPriorityTaskWoken;
  uint32_t ulCurrBuf = 0;
  //SPI1 Stream
  if (DMA1->LISR & DMA_FLAG_TCIF1_5){
      uint32_t status = DMA1_Stream1->CR;
	if ((status & DMA_SxCR_CT_Msk)!=0){ //current Transmittable Buffer
	ulCurrBuf = 0;
	} else {
	ulCurrBuf = 1;
	}

    }
  /* USER CODE END DMA1_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi2_rx);
  /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */
  /* xHigherPriorityTaskWoken must be initialised to pdFALSE. If calling
       xTaskNotifyFromISR() unblocks the handling task, and the priority of
       the handling task is higher than the priority of the currently running task,
       then xHigherPriorityTaskWoken will automatically get set to pdTRUE. */
    xHigherPriorityTaskWoken = pdFALSE;
    /* Unblock the handling task so the task can perform any processing necessitated
       by the interrupt. xHandlingTask is the task's handle, which was obtained
       when the task was created. The handling task's 0th notification value
       is bitwise ORed with the interrupt status - ensuring bits that are already
       set are not overwritten. */
    xTaskNotifyFromISR( vAuxADCTCPTaskHandle,
  			     ulCurrBuf,
  			     eSetBits,
  			     &xHigherPriorityTaskWoken );

    /* Force a context switch if xHigherPriorityTaskWoken is now set to pdTRUE.
       The macro used to do this is dependent on the port and may be called
       portEND_SWITCHING_ISR. */
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
  /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream2 global interrupt.
  */
void DMA1_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream2_IRQn 0 */

  /* USER CODE END DMA1_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_tim1_up);
  /* USER CODE BEGIN DMA1_Stream2_IRQn 1 */

  /* USER CODE END DMA1_Stream2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream3 global interrupt.
  */
void DMA1_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream3_IRQn 0 */

  /* USER CODE END DMA1_Stream3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_tim3_up);
  /* USER CODE BEGIN DMA1_Stream3_IRQn 1 */

  /* USER CODE END DMA1_Stream3_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream4 global interrupt.
  */
void DMA1_Stream4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream4_IRQn 0 */

  /* USER CODE END DMA1_Stream4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_tim4_ch1);
  /* USER CODE BEGIN DMA1_Stream4_IRQn 1 */

  /* USER CODE END DMA1_Stream4_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream5 global interrupt.
  */
void DMA1_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream5_IRQn 0 */

  /* USER CODE END DMA1_Stream5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_tim4_up);
  /* USER CODE BEGIN DMA1_Stream5_IRQn 1 */

  /* USER CODE END DMA1_Stream5_IRQn 1 */
}

/**
  * @brief This function handles SPI1 global interrupt.
  */
void SPI1_IRQHandler(void)
{
  /* USER CODE BEGIN SPI1_IRQn 0 */

  /* USER CODE END SPI1_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi1);
  /* USER CODE BEGIN SPI1_IRQn 1 */

  /* USER CODE END SPI1_IRQn 1 */
}

/**
  * @brief This function handles SPI2 global interrupt.
  */
void SPI2_IRQHandler(void)
{
  /* USER CODE BEGIN SPI2_IRQn 0 */

  /* USER CODE END SPI2_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi2);
  /* USER CODE BEGIN SPI2_IRQn 1 */

  /* USER CODE END SPI2_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
